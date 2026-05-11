import numpy as np

from manim import *
from manim_themes.manim_theme import apply_theme


def func(x):
    return x**2


class ParabelStability(Scene):
    def setup(self):
        apply_theme(manim_scene=self, theme_name="Monokai Pro Light", light_theme=True)

    def construct(self):
        # Animation setup
        keyframes = [0, 1, 0, -1, 0]
        t = ValueTracker(keyframes[0])

        # PoS - init
        PoS = Rectangle(height=3, width=6)
        PoS.set_fill(BLUE, opacity=0.3)
        PoS.to_edge(LEFT)
        marker_PoS = Dot().move_to(PoS)
        self.add(PoS, marker_PoS)

        # PoS - animation
        marker_PoS.add_updater(
            lambda x: x.move_to(
                PoS.get_center() + np.array([0, PoS.height * 0.5 * t.get_value(), 0])
            )
        )

        # Graph - init
        ax = Axes(
            x_range=[-1.2, 1.2, 0.5],
            y_range=[0, 1.2, 0.5],
            x_length=8,
            y_length=5,
            y_axis_config={
                "numbers_to_include": np.arange(0, 1.1, 0.5),
            },
            tips=False,
        ).to_edge(RIGHT)
        labels = ax.get_axis_labels(x_label=r"P", y_label=r"l_s")
        graph = ax.plot(func, x_range=[-1, 1], color=MAROON)

        lower_line = ax.get_vertical_line(ax.i2gp(-1, graph), color=GRAY)
        upper_line = ax.get_vertical_line(ax.i2gp(1, graph), color=GRAY)

        # Graph - animation
        pt0 = [ax.coords_to_point(t.get_value(), func(t.get_value()))]
        dot_graph = Dot(point=pt0)
        dot_graph.add_updater(
            lambda x: x.move_to((ax.c2p(t.get_value(), func(t.get_value()))))
        )
        self.add(ax, labels, graph, dot_graph, lower_line, upper_line)

        # run animation
        for k in keyframes[1:]:
            self.play(t.animate.set_value(k))
            self.wait(1)
