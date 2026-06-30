#!/usr/bin/env python3
"""
acados_iterate_visualizer.py
=============================

Interactive, terminal-only visualizer for the per-iteration data of an
``AcadosOcpSolver`` SQP solve. Built on ``plotext`` (terminal plotting)
+ ``readchar`` (raw key capture) + optionally ``rich`` (flicker-free
redraws, see https://github.com/piccolomo/plotext/blob/master/readme/environments.md#rich).

Workflow
--------
1. A field menu is shown (x, u, z, lam, pi, sl, su, or "residuals").
   Navigate with Up/Down (or j/k), confirm with Enter, quit with q/Esc.
2. Once a field is chosen, you move through the SQP iterations:
       Left/Right (or h/l)  : step one iteration back / forward
       Space                : play / pause auto-advance
       g <digits> Enter     : jump to a specific iteration
       q / Esc               : back to the field menu

Requirements
------------
    pip install plotext readchar
    pip install rich        # optional, gives flicker-free redraws

Important notes on the acados side
-----------------------------------
* Per-iteration data is NOT part of ``AcadosOcp`` -- it only exists on the
  ``AcadosOcpSolver`` after a solve, and only if intermediate iterates
  were kept in memory. Set this *before* creating/solving:

      ocp.solver_options.store_iterates = True
      ocp_solver = AcadosOcpSolver(ocp, ...)
      ocp_solver.solve()
      visualize_ocp_solver(ocp_solver)

* This module calls ``ocp_solver.get_iterate(i)`` for every stored SQP
  iteration ``i`` (0 ... nlp_iter), and reads the ``x_traj``, ``u_traj``,
  ``z_traj``, ``lam_traj``, ``pi_traj``, ``sl_traj``, ``su_traj``
  attributes of the returned ``AcadosOcpIterate`` -- each is a list with
  one array per shooting stage.
* For the "residuals" view it calls ``ocp_solver.get_stats('statistics')``,
  which returns the same table printed by ``print_statistics()``
  (iter, res_stat, res_eq, res_ineq, res_comp, qp_stat, qp_iter, alpha, ...).
  The exact set of columns can vary with acados version / solver options
  (e.g. SQP_RTI, ``nlp_solver_ext_qp_res``), so this module only labels as
  many leading columns as it recognizes and falls back to generic names
  ("field_4", ...) for the rest -- nothing is dropped.

This file is self-testing: running it directly (``python
acados_iterate_visualizer.py``) drives the UI against a small synthetic
mock solver, so you can try the controls without acados installed.
"""

from __future__ import annotations

import queue
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import plotext as plt
import readchar

try:
    from rich.live import Live
    from rich.text import Text
    _HAS_RICH = True
except ImportError:
    _HAS_RICH = False


# --------------------------------------------------------------------------
# Field definitions
# --------------------------------------------------------------------------

# field name -> (AcadosOcpIterate attribute, human-readable description)
ITERATE_FIELDS: Dict[str, Tuple[str, str]] = {
    "x":   ("x_traj",   "state trajectory x"),
    "u":   ("u_traj",   "control trajectory u"),
    "z":   ("z_traj",   "algebraic state trajectory z"),
    "lam": ("lam_traj", "inequality multipliers lam"),
    "pi":  ("pi_traj",  "equality multipliers pi"),
    "sl":  ("sl_traj",  "lower slacks sl"),
    "su":  ("su_traj",  "upper slacks su"),
}
RESIDUALS_KEY = "residuals (SQP statistics)"

# leading columns of get_stats('statistics'), in the order acados emits them
# (see AcadosOcpSolver.print_statistics() docstring). Extra columns that
# exist for some solver settings (e.g. qp_res_*) are labeled generically.
KNOWN_STAT_COLUMNS = [
    "iter", "res_stat", "res_eq", "res_ineq", "res_comp",
    "qp_stat", "qp_iter", "alpha",
    "qp_res_stat", "qp_res_eq", "qp_res_ineq", "qp_res_comp",
]
RESIDUAL_CURVES = ["res_stat", "res_eq", "res_ineq", "res_comp"]


# --------------------------------------------------------------------------
# Data extraction from the solver
# --------------------------------------------------------------------------

def _get_num_iterations(solver) -> int:
    """Number of SQP/NLP iterations performed in the last solve() call."""
    try:
        return int(solver.get_stats("nlp_iter"))
    except Exception:
        # SQP_RTI and similar solvers may not expose 'nlp_iter'
        return int(solver.get_stats("sqp_iter"))


def _stack_stage_trajectory(traj: List[np.ndarray]) -> np.ndarray:
    """
    Turns a list of per-stage 1D arrays (possibly of differing length,
    e.g. terminal stage has no control) into a single
    (n_stages, max_dim) matrix, padded with NaN.
    """
    arrays = [np.atleast_1d(np.asarray(v, dtype=float)) for v in traj]
    max_dim = max((a.shape[0] for a in arrays), default=0)
    mat = np.full((len(arrays), max_dim), np.nan)
    for s, a in enumerate(arrays):
        mat[s, : a.shape[0]] = a
    return mat


def load_field_over_iterations(solver, field: str) -> List[np.ndarray]:
    """
    Returns a list, indexed by SQP iteration (0 ... nlp_iter), of
    (n_stages, dim) matrices for the requested iterate field.

    Requires ocp.solver_options.store_iterates = True to have been set
    before solve().
    """
    attr_name, _ = ITERATE_FIELDS[field]
    n_iter = _get_num_iterations(solver)
    data = []
    for k in range(n_iter + 1):
        try:
            iterate = solver.get_iterate(k)
        except Exception as exc:
            raise RuntimeError(
                f"Could not retrieve iterate {k}. Make sure "
                "ocp.solver_options.store_iterates = True was set BEFORE "
                f"calling solve(). Original error: {exc}"
            ) from exc
        traj = getattr(iterate, attr_name)
        data.append(_stack_stage_trajectory(traj))
    return data


def load_residual_statistics(solver) -> Dict[str, np.ndarray]:
    """
    Returns dict: column name -> 1D np.array over SQP iterations,
    parsed from get_stats('statistics').
    """
    stats = np.asarray(solver.get_stats("statistics"), dtype=float)
    n_iter = _get_num_iterations(solver)

    if stats.ndim != 2:
        raise RuntimeError(
            f"Unexpected shape for get_stats('statistics'): {stats.shape}"
        )

    # acados emits this as (n_fields, n_iter+1); be defensive about
    # orientation in case that ever changes.
    if stats.shape[0] == n_iter + 1 and stats.shape[1] != n_iter + 1:
        stats = stats.T

    n_fields = stats.shape[0]
    if n_fields <= len(KNOWN_STAT_COLUMNS):
        names = KNOWN_STAT_COLUMNS[:n_fields]
    else:
        names = KNOWN_STAT_COLUMNS + [
            f"field_{i}" for i in range(len(KNOWN_STAT_COLUMNS), n_fields)
        ]
    return {name: stats[i, :] for i, name in enumerate(names)}


# --------------------------------------------------------------------------
# Key listener (runs in a background thread so we can support a
# play/pause auto-advance mode while still reacting to key presses)
# --------------------------------------------------------------------------

def _start_key_listener() -> "queue.Queue[str]":
    q: "queue.Queue[str]" = queue.Queue()

    def _worker():
        while True:
            try:
                key = readchar.readkey()
            except Exception:
                break
            q.put(key)

    threading.Thread(target=_worker, daemon=True).start()
    return q


# --------------------------------------------------------------------------
# Display abstraction: rich.Live if available (flicker-free), else a
# plain "clear terminal and reprint" fallback.
# --------------------------------------------------------------------------

class _Display:
    def __enter__(self):
        if _HAS_RICH:
            self._live = Live(refresh_per_second=12, screen=True)
            self._live.__enter__()
        return self

    def __exit__(self, *exc):
        if _HAS_RICH:
            self._live.__exit__(*exc)

    def update(self, plot_text: str) -> None:
        if _HAS_RICH:
            self._live.update(Text.from_ansi(plot_text))
        else:
            plt.clear_terminal()
            print(plot_text)


# --------------------------------------------------------------------------
# Rendering helpers
# --------------------------------------------------------------------------

def _status_line(title: str, idx: int, n_iter: int, playing: bool,
                  goto_buffer: Optional[str], help_text: str) -> str:
    status = "PLAYING" if playing else "PAUSED"
    line = f"{title}  |  SQP iter {idx}/{n_iter}  |  {status}"
    if goto_buffer is not None:
        line += f"  |  goto: {goto_buffer}_"
    return line + "\n" + help_text


def _render_iterate_frame(field: str, mat: np.ndarray, idx: int, n_iter: int,
                           playing: bool, goto_buffer: Optional[str]) -> str:
    plt.clear_figure()
    n_stages, n_dim = mat.shape
    x_labels = [str(s) for s in range(n_stages)]

    dims_with_data = [d for d in range(n_dim) if not np.all(np.isnan(mat[:, d]))]
    if not dims_with_data:
        dims_with_data = [0] if n_dim > 0 else []

    if len(dims_with_data) <= 1:
        d = dims_with_data[0] if dims_with_data else 0
        heights = np.nan_to_num(mat[:, d]).tolist() if n_dim > 0 else [0.0] * n_stages
        plt.bar(x_labels, heights, label=field)
    else:
        # one group of bars per shooting stage, one bar per field dimension
        heights_per_dim = [np.nan_to_num(mat[:, d]).tolist() for d in dims_with_data]
        labels = [f"{field}[{d}]" for d in dims_with_data]
        plt.multiple_bar(x_labels, heights_per_dim, labels=labels)

    plt.xlabel("shooting stage")
    plt.ylabel(field)
    plt.title(f"{field}  -  SQP iteration {idx}/{n_iter}")
    body = plt.build()

    help_text = ("animates automatically (1s/iteration)   "
                 "\u2190/\u2192 or h/l: step manually   space: play/pause   "
                 "g+digits+enter: goto   q/esc: back to menu")
    return body + "\n" + _status_line(field, idx, n_iter, playing, goto_buffer, help_text)


def _render_residual_frame(residuals: Dict[str, np.ndarray], idx: int, n_iter: int,
                            playing: bool, goto_buffer: Optional[str]) -> str:
    plt.clear_figure()
    names = [n for n in RESIDUAL_CURVES if n in residuals]
    heights = [max(float(residuals[n][idx]), 1e-16) for n in names]  # keep log-scale sane
    plt.bar(names, heights, color=["blue+", "green+", "orange+", "red+"][: len(names)],
            minimum=1e-12)
    plt.yscale("log")
    plt.xlabel("residual type")
    plt.ylabel("value (log scale)")
    plt.title(f"NLP residual statistics  -  SQP iteration {idx}/{n_iter}")
    body = plt.build()

    cursor_values = ", ".join(f"{n}={residuals[n][idx]:.3e}" for n in names)
    help_text = ("animates automatically (1s/iteration)   "
                 "\u2190/\u2192 or h/l: step manually   space: play/pause   "
                 "g+digits+enter: goto   q/esc: back to menu\n"
                 f"at iter {idx}: {cursor_values}")
    return body + "\n" + _status_line("residuals", idx, n_iter, playing, goto_buffer, help_text)


# --------------------------------------------------------------------------
# Generic "step through iterations" loop, shared by iterate fields and
# the residuals view.
# --------------------------------------------------------------------------

ANIMATION_INTERVAL_SECONDS = 1.0  # time each SQP iteration is shown during autoplay


def _browse(key_queue: "queue.Queue[str]", n_iter: int, render_fn,
            autoplay: bool = True) -> None:
    idx = 0
    playing = autoplay and n_iter > 0
    goto_buffer: Optional[str] = None

    with _Display() as disp:
        while True:
            disp.update(render_fn(idx, playing, goto_buffer))

            timeout = ANIMATION_INTERVAL_SECONDS if playing else None
            try:
                key = key_queue.get(timeout=timeout)
            except queue.Empty:
                key = None

            if key is None and playing:
                if idx < n_iter:
                    idx += 1
                else:
                    playing = False
                continue

            if goto_buffer is not None:
                if key in (readchar.key.ENTER, readchar.key.CR, readchar.key.LF):
                    if goto_buffer != "":
                        idx = max(0, min(n_iter, int(goto_buffer)))
                    goto_buffer = None
                elif key == readchar.key.ESC:
                    goto_buffer = None
                elif key in (readchar.key.BACKSPACE, readchar.key.DELETE):
                    goto_buffer = goto_buffer[:-1]
                elif isinstance(key, str) and key.isdigit():
                    goto_buffer += key
                continue

            if key in (readchar.key.LEFT, "h"):
                idx, playing = max(0, idx - 1), False
            elif key in (readchar.key.RIGHT, "l"):
                idx, playing = min(n_iter, idx + 1), False
            elif key == readchar.key.SPACE:
                playing = not playing
            elif key in ("g", "G"):
                playing, goto_buffer = False, ""
            elif key in (readchar.key.ESC, "q", "b"):
                return


# --------------------------------------------------------------------------
# Field selection menu
# --------------------------------------------------------------------------

def _select_from_list(options: List[str], key_queue: "queue.Queue[str]",
                       title: str) -> Optional[int]:
    selected = 0
    with _Display() as disp:
        while True:
            lines = [title, ""]
            for i, opt in enumerate(options):
                marker = ">" if i == selected else " "
                lines.append(f" {marker} {opt}")
            lines.append("")
            lines.append("Up/Down or j/k: move   Enter: select   q/esc: quit")
            disp.update("\n".join(lines))

            key = key_queue.get()
            if key in (readchar.key.UP, "k"):
                selected = (selected - 1) % len(options)
            elif key in (readchar.key.DOWN, "j"):
                selected = (selected + 1) % len(options)
            elif key in (readchar.key.ENTER, readchar.key.CR, readchar.key.LF):
                return selected
            elif key in (readchar.key.ESC, "q"):
                return None


# --------------------------------------------------------------------------
# Public entry point
# --------------------------------------------------------------------------

def visualize_ocp_solver(solver) -> None:
    """
    Launch the interactive terminal visualizer for ``solver``.

    :param solver: an ``AcadosOcpSolver`` that has already been solved with
        ``ocp.solver_options.store_iterates = True``.
    """
    n_iter = _get_num_iterations(solver)
    menu_items = list(ITERATE_FIELDS.keys()) + [RESIDUALS_KEY]
    key_queue = _start_key_listener()

    while True:
        choice = _select_from_list(
            menu_items, key_queue,
            title=f"AcadosOcp iterate visualizer  ({n_iter} stored SQP iterations)",
        )
        if choice is None:
            break

        field = menu_items[choice]
        if field == RESIDUALS_KEY:
            residuals = load_residual_statistics(solver)
            _browse(
                key_queue, n_iter,
                lambda idx, playing, gb, _r=residuals: _render_residual_frame(_r, idx, n_iter, playing, gb),
            )
        else:
            data = load_field_over_iterations(solver, field)
            _browse(
                key_queue, n_iter,
                lambda idx, playing, gb, _d=data, _f=field: _render_iterate_frame(_f, _d[idx], idx, n_iter, playing, gb),
            )

    print("Bye!")


# --------------------------------------------------------------------------
# Self-test / demo with a synthetic mock solver (no acados required)
# --------------------------------------------------------------------------

class _MockIterate:
    def __init__(self, x_traj, u_traj, lam_traj, pi_traj):
        self.x_traj = x_traj
        self.u_traj = u_traj
        self.z_traj = [np.zeros(0) for _ in x_traj]
        self.lam_traj = lam_traj
        self.pi_traj = pi_traj
        self.sl_traj = [np.zeros(0) for _ in x_traj]
        self.su_traj = [np.zeros(0) for _ in x_traj]


class _MockSolver:
    """Implements just enough of the AcadosOcpSolver surface for this demo."""

    def __init__(self, n_stages=20, nx=4, nu=2, n_sqp_iter=12, seed=0):
        rng = np.random.default_rng(seed)
        self.N = n_stages
        self.n_sqp_iter = n_sqp_iter
        x_target = rng.normal(size=nx)
        u_target = rng.normal(size=nu)
        self._iterates = []
        for k in range(n_sqp_iter + 1):
            blend = k / n_sqp_iter
            noise = (1 - blend) * 2.0
            x_traj = [x_target * blend + rng.normal(scale=noise, size=nx) for _ in range(n_stages + 1)]
            u_traj = [u_target * blend + rng.normal(scale=noise, size=nu) for _ in range(n_stages)]
            lam_traj = [np.abs(rng.normal(scale=noise * 0.5, size=2 * nu)) for _ in range(n_stages)] + \
                       [np.abs(rng.normal(scale=noise * 0.5, size=2 * nx))]
            pi_traj = [rng.normal(scale=noise, size=nx) for _ in range(n_stages)]
            self._iterates.append(_MockIterate(x_traj, u_traj, lam_traj, pi_traj))

        res = np.array([(1 - k / n_sqp_iter) ** 2 + 1e-8 for k in range(n_sqp_iter + 1)])
        self._stats = np.vstack([
            np.arange(n_sqp_iter + 1),
            res, res * 1.3, res * 0.7, res * 0.2,
            np.zeros(n_sqp_iter + 1),
            rng.integers(1, 8, size=n_sqp_iter + 1),
            np.ones(n_sqp_iter + 1),
        ])

    def get_iterate(self, k):
        return self._iterates[k]

    def get_stats(self, field):
        if field == "nlp_iter":
            return self.n_sqp_iter
        if field == "statistics":
            return self._stats
        raise KeyError(field)


def _self_test() -> None:
    """Non-interactive sanity check: exercises data loading + rendering."""
    solver = _MockSolver()
    for field in ITERATE_FIELDS:
        data = load_field_over_iterations(solver, field)
        assert len(data) == solver.n_sqp_iter + 1
        _render_iterate_frame(field, data[0], 0, solver.n_sqp_iter, False, None)
    residuals = load_residual_statistics(solver)
    _render_residual_frame(residuals, 0, solver.n_sqp_iter, False, None)
    print("Self-test OK: data loading and rendering succeeded for all fields.")


if __name__ == "__main__":
    import sys

    if "--selftest" in sys.argv:
        _self_test()
    else:
        print("Running interactive demo with synthetic data "
              "(no acados installation required).")
        visualize_ocp_solver(_MockSolver())