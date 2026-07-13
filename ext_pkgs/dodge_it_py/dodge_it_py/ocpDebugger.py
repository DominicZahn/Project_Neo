from dataclasses import dataclass
from itertools import cycle
import yaml
import numpy as np
import numpy.typing as npt
from pathlib import Path
from typing_extensions import Literal

from textual.app import App, ComposeResult
from textual.containers import (
    Grid, VerticalGroup, VerticalScroll, HorizontalGroup,
)
from textual.binding import Binding
from textual.widgets import (
    Footer, Header, Button, Label, ListView, ListItem,
    Static, Rule
)
from textual.theme import Theme
from textual.screen import Screen, ModalScreen
from textual.reactive import reactive
from textual_plotext import PlotextPlot

from acados_template import AcadosOcpSolver, AcadosOcpIterate, AcadosModel

@dataclass
class AcadosField:
    var: str
    type: Literal["NLP", "QP"]
    description: str

class VimListView(ListView):
    BINDINGS = [
        Binding("k", "cursor_up", "Cursor up", show=False),
        Binding("j", "cursor_down", "Cursor down", show=False),
    ]


class FieldSelectorScreen(ModalScreen[AcadosField]):

    def __init__(self, file_path : Path) -> None:
        super().__init__(id="FieldSelectorScreen")
        valid_file = file_path.exists() and file_path.is_file() and file_path.suffix == ".yaml"
        if not valid_file:
            self.notify(
                str(file_path)+" is not a valid acados field config!",
                severity="error")
        
        # import fields
        yaml_stream = open(file_path, "r")
        yaml_data_list = yaml.safe_load_all(yaml_stream)
        self.fields = [AcadosField(**d) for d in yaml_data_list if d is not None]

    @staticmethod
    def field2Vis(field : AcadosField) -> ListItem:
        return ListItem(
            Grid(
                Static(field.var),
                Static(field.type),
                Static(field.description),
                classes="FieldSelectorItem"
            )
        )

    def compose(self) -> ComposeResult:
        yield Grid(
            Static("VAR", classes="FieldSelectorHeader"),
            Static("TYPE", classes="FieldSelectorHeader"),
            Static("DESCRIPTION", classes="FieldSelectorHeader"),
            classes="FieldSelectorItem"
        )

        list_items = list(map(self.field2Vis, self.fields))
        field_list = VimListView(*list_items, classes="FieldSelectorList")
        yield field_list

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        self.dismiss(self.fields[event.index])

class PlotterLegend(VimListView):
    def __init__(self, field : AcadosField, dims : list[str], color_cycle : list[str]) -> None:
        super().__init__()
        self.field = field
        self.dims = dims
        self.cycle_colors = color_cycle
        self.classes = "PlotterLegend"

    def on_mount(self) -> None:
        for dim, color in zip(self.dims, cycle(self.cycle_colors)):
            label = Label(dim)
            label.styles.color = color
            self.mount(ListItem(label))

class Plotter(HorizontalGroup):

#    COLOR_CYCLE = [
#        "green",
#        "violet",
#        "white",
#        "cyan",
#        "tomato" ]

    COLOR_CYCLE = [
        "green",
        "red",
        "white",
        "cyan"]


    def __init__(self,
                 field : AcadosField,
                 solver_data : npt.NDArray) -> None:
        super().__init__()
        self.field = field
        self.solver_data = solver_data
        self.index = 0

    def redraw(self) -> None:
        self.plot.plt.clear_data()

        for i, data in enumerate(self.solver_data):
            data = data.tolist()
            color = self.COLOR_CYCLE[i % len(self.COLOR_CYCLE)]
            if i == self.index:
                self.plot.plt.plot(data, marker='braille', color=color)
            self.plot.plt.scatter(data, marker='braille', color=color)
        self.plot.refresh()

    def compose(self) -> ComposeResult:
        self.plot = PlotextPlot()
        self.redraw()
        self.plot.plt.title(self.field.var)
        self.plot.plt.xlabel("Shooting Nodes")
        yield self.plot
        dims = list(map(str, range(len(self.solver_data))))
        legend = PlotterLegend(self.field, dims, self.COLOR_CYCLE)
        yield legend

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        self.index = event.index
        self.redraw()

class OcpDebugger(App):
    """
    A textual app to investigate the results of an AcadosOcpSolver run.
    """

    TITLE = "Acados OCP Debugger"
    CSS_PATH = "ocpDebugger.tcss"
    BINDINGS = [
        ("d", "toggle_dark", "Toggle dark mode"),
        ("a", "add", "Add"),
        ("r", "remove", "Remove"),
        ("h", "prev_iter", "Previous Iteration"),
        ("l", "next_iter", "Next Iteration"),
    ]

    ACADOS_FIELD_CONFIG = Path("/home/robot/ws/ext_pkgs/dodge_it_py/dodge_it_py/acados_fields.yaml")

    def __init__(self, solver : AcadosOcpSolver) -> None:
        super().__init__()
        self.solver = solver
        self.max_iter = int(solver.get_stats("nlp_iter"))
        self.iter_i = self.max_iter

        theme = self.get_theme("monokai")
        assert(theme)
        self.register_theme(theme)
        self.theme = "monokai"
        self.ansi_color = False

    def compose(self) -> ComposeResult:
        yield Header()
        yield Grid(
            id="plotter_grid"
        )
        yield HorizontalGroup(
            Footer(),
            Label(f"iter: {self.iter_i:03}", id="iter_counter"),
            id="advanced_footer"
        )

    def action_add(self) -> None:
        def get_field(field : AcadosField | None):
            assert(field)
            data = self._retrieve_data(field.var)
            if (data is None):
                return 
            new_plotter = Plotter(field, data)
            self.query_one("#plotter_grid").mount(new_plotter)
            new_plotter.scroll_visible()

        self.push_screen(
            FieldSelectorScreen(self.ACADOS_FIELD_CONFIG),
            get_field)

    def action_remove(self) -> None:
       plotters = self.query("Plotter")
       if plotters:
           plotters.last().remove()

    def action_toggle_dark(self) -> None:
        return super().action_toggle_dark()

    def _retrieve_data(self, field_var : str) -> npt.NDArray | None:
        iter = self.solver.get_iterate(self.iter_i)
        if not hasattr(iter, field_var):
            msg = field_var + " is not available!"
            self.notify(msg,
                severity="error")
            return None
        data = getattr(iter, field_var)
        if field_var == "lam": # removing lam_0 und lam_e as they could be inconsistent
            size = data[1].shape[0]
            data[0] = np.zeros(size)
            data[-1] = np.zeros(size)
        return np.array(data).transpose()
    
    def _update_for_iter(self, wraparound=False) -> None:
        # update counter
        iter_counter = self.query_one("#iter_counter", Label)
        iter_counter.update(f"iter: {self.iter_i:03}")
        style_class = "highlight_iter_wraparound" if wraparound else "highlight_iter"
        iter_counter.add_class(style_class)
        self.set_timer(
            0.2,
            lambda w=iter_counter: w.remove_class(style_class)
        )

        # update data
        w_list = self.query_one("#plotter_grid", Grid).children
        for w in w_list:
            if type(w) is not Plotter:
                continue
            data = self._retrieve_data(w.field.var)
            assert(data is not None)
            w.solver_data = data
            w.redraw()

    def action_next_iter(self) -> None:
        self.iter_i += 1
        wraparound = self.iter_i > self.max_iter
        if wraparound:
            self.iter_i = 0
        self._update_for_iter(wraparound)

    def action_prev_iter(self) -> None:
        self.iter_i -= 1
        wraparound = self.iter_i < 0
        if wraparound:
            self.iter_i = self.max_iter
        self._update_for_iter(wraparound)