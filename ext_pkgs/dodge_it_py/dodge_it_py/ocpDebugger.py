from time import monotonic
from dataclasses import dataclass

from textual.app import App, ComposeResult
from textual.containers import (
    Grid, VerticalGroup, HorizontalGroup
)
from textual.widgets import (
    Footer, Header, Button, Label
)
from textual.screen import Screen, ModalScreen
from textual.reactive import reactive
from textual_plotext import PlotextPlot

from acados_template import AcadosOcpSolver, AcadosOcpIterate, AcadosModel

@dataclass
class Field:
    name: str

class FieldSelectorScreen(ModalScreen[str]):

    def compose(self) -> ComposeResult:
        yield VerticalGroup(
            Label("Which field do you want to monitor?", id="field_label"),
            Button("Quit", id="quit_button"),
            id="field_selector_grid"
        )

    def on_button_pressed(self, event : Button.Pressed) -> None:
        bt_id = event.button.id
        if bt_id == "quit_button":
            self.dismiss("test field")

class Plotter(PlotextPlot):

    def on_mount(self) -> None:
        y = self.plt.sin()
        self.plt.scatter(y, marker="braille")
        self.plt.title("Test Title")

class OcpDebugger(App):
    """
    A textual app to investigate the results of an AcadosOcpSolver run.
    """

    TITLE = "Acados OCP Debugger"
    CSS_PATH = "ocpDebugger.tcss"
    BINDINGS = [
        ("d", "toggle_dark", "Toggle dark mode"),
        ("a", "add", "Add"),
        ("r", "remove", "Remove")
    ]

    def __init__(self, solver : AcadosOcpSolver) -> None:
        super().__init__()
        self.solver = solver
        self.iter_i = reactive(0)

        assert(self.solver.acados_ocp)
        assert(self.solver.acados_ocp.model)
        assert(type(self.solver.acados_ocp.model) is AcadosModel)
        self.nu = self.solver.acados_ocp.model.u.size1()
        self.nx = self.solver.acados_ocp.model.x.size1()

        # TODO

    def compose(self) -> ComposeResult:
        yield Header()
        yield Grid(
            Plotter(),
            Plotter(),
            Plotter(),
            id="plotter_grid"
        )
        yield Footer()

    def action_add(self) -> None:
        def get_field(field : str | None):
            assert(field)
            self.notify(field)
        self.push_screen(FieldSelectorScreen(), get_field)

        new_plotter = Plotter()
        self.query_one("#plotter_grid").mount(new_plotter)
        # new_plotter.scroll_visible()

    def action_remove(self) -> None:
        timers = self.query("Plotter")
        if timers:
            timers.last().remove()

    def action_toggle_dark(self) -> None:
        return super().action_toggle_dark()
