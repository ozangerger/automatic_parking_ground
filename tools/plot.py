#!/usr/bin/env python3
"""Plot one simulation run written by CsvSink and write_scene.

Usage:
    python3 tools/plot.py RUN_DIR [-o OUT.png] [--dark] [--show]

RUN_DIR holds run.csv (one row per tick, columns as documented in csv_sink.hpp) and, optionally,
scene.json (obstacles, slot, car footprint). Panels: top-down trajectory of truth and estimate,
position error and yaw error against time, and NIS against time when run.csv has an `nis` column.

Conventions match the simulator: SI units, x forward/east, y left/north, yaw counter-clockwise.
Matplotlib axes already have y pointing up, so world coordinates are plotted unchanged; the one
place where a frame changes is `car_in_world`.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

import numpy as np

# Categorical slots 1 and 2 of the reference palette, validated for light and dark surfaces
# (scripts/validate_palette.py: CVD dE 24.7 / 26.8, normal-vision dE 33.6 / 31.8, contrast >= 3:1).
THEMES = {
    "light": {
        "surface": "#fcfcfb",
        "ink": "#0b0b0b",
        "ink_secondary": "#52514e",
        "grid": "#e4e3df",
        "obstacle_fill": "#d6d5d0",
        "obstacle_edge": "#9a9992",
        "truth": "#2a78d6",
        "estimate": "#eb6834",
    },
    "dark": {
        "surface": "#1a1a19",
        "ink": "#f0efec",
        "ink_secondary": "#c3c2b7",
        "grid": "#33322f",
        "obstacle_fill": "#3a3936",
        "obstacle_edge": "#6e6d67",
        "truth": "#3987e5",
        "estimate": "#d95926",
    },
}

LINE_WIDTH = 1.8
MARKER_SIZE = 8


def load_run(path):
    """Returns {column: np.ndarray of float}; empty cells become NaN, text columns are dropped."""
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    columns = {}
    for name in rows[0].keys():
        values = []
        try:
            for row in rows:
                cell = row[name]
                values.append(float(cell) if cell != "" else math.nan)
        except ValueError:
            continue  # text column such as cmd_gear
        columns[name] = np.array(values)
    return columns


def load_scene(path):
    if not path.exists():
        return {}
    with open(path) as handle:
        return json.load(handle)


def car_in_world(footprint, x, y, yaw):
    """Vehicle-frame footprint corners [[x, y], ...] placed at the rear axle pose (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in footprint]


def style_axes(ax, theme):
    ax.set_facecolor(theme["surface"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(theme["ink_secondary"])
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=theme["ink_secondary"], labelsize=9, length=3)
    ax.grid(True, color=theme["grid"], linewidth=0.7)
    ax.set_axisbelow(True)
    ax.xaxis.label.set_color(theme["ink_secondary"])
    ax.yaxis.label.set_color(theme["ink_secondary"])


def draw_trajectory(ax, run, scene, theme):
    from matplotlib.patches import Polygon as PolygonPatch

    for obstacle in scene.get("obstacles", []):
        ax.add_patch(PolygonPatch(obstacle["vertices"], closed=True, facecolor=theme["obstacle_fill"],
                                  edgecolor=theme["obstacle_edge"], linewidth=0.8, zorder=1))
    slot = scene.get("slot")
    if slot:
        ax.add_patch(PolygonPatch(slot["vertices"], closed=True, fill=False,
                                  edgecolor=theme["ink_secondary"], linewidth=1.0, linestyle=(0, (4, 3)),
                                  zorder=2))

    ax.plot(run["truth_x"], run["truth_y"], color=theme["truth"], linewidth=LINE_WIDTH,
            label="truth", zorder=4)
    ax.plot(run["est_x"], run["est_y"], color=theme["estimate"], linewidth=LINE_WIDTH,
            linestyle=(0, (5, 2)), label="estimate", zorder=5)

    # Start and end of the truth path; the car outline shows heading and size at both poses.
    footprint = scene.get("car")
    for index, name in ((0, "start"), (-1, "end")):
        x, y, yaw = run["truth_x"][index], run["truth_y"][index], run["truth_yaw"][index]
        ax.plot([x], [y], marker="o", markersize=MARKER_SIZE, color=theme["truth"],
                markeredgecolor=theme["surface"], markeredgewidth=2, linestyle="none", zorder=6)
        if footprint:
            ax.add_patch(PolygonPatch(car_in_world(footprint, x, y, yaw), closed=True, fill=False,
                                      edgecolor=theme["ink_secondary"], linewidth=0.9, zorder=3))
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(8, 8), fontsize=9,
                    color=theme["ink_secondary"])

    ax.set_aspect("equal", adjustable="datalim")
    ax.autoscale_view()
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectory, top-down", loc="left", fontsize=11, color=theme["ink"])
    legend = ax.legend(loc="best", frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(theme["ink_secondary"])


def draw_series(ax, t, values, theme, color, title, ylabel, zero_line=False, sparse=False):
    """Line plot of `values`. With sparse=True only the finite samples are drawn, as small
    markers joined by a thin line (events such as filter updates leave blank cells in between)."""
    if sparse:
        keep = np.isfinite(values)
        ax.plot(t[keep], values[keep], color=theme[color], linewidth=0.8, marker="o",
                markersize=3.5, markeredgewidth=0)
    else:
        ax.plot(t, values, color=theme[color], linewidth=LINE_WIDTH)
    if zero_line:
        ax.axhline(0.0, color=theme["ink_secondary"], linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=11, color=theme["ink"])


def make_figure(run, scene, theme, title):
    import matplotlib.pyplot as plt

    has_nis = "nis" in run and np.isfinite(run["nis"]).any()
    panels = 3 if has_nis else 2

    fig = plt.figure(figsize=(12, 6.5), facecolor=theme["surface"])
    grid = fig.add_gridspec(panels, 2, width_ratios=(1.5, 1.0), hspace=0.55, wspace=0.22,
                            left=0.06, right=0.97, top=0.83, bottom=0.09)

    ax_traj = fig.add_subplot(grid[:, 0])
    style_axes(ax_traj, theme)
    draw_trajectory(ax_traj, run, scene, theme)

    t = run["t_s"]
    ax_pos = fig.add_subplot(grid[0, 1])
    style_axes(ax_pos, theme)
    draw_series(ax_pos, t, run["pos_err"], theme, "estimate", "Position error (estimate vs truth)",
                "distance (m)")
    ax_pos.set_ylim(bottom=0.0)

    ax_yaw = fig.add_subplot(grid[1, 1], sharex=ax_pos)
    style_axes(ax_yaw, theme)
    draw_series(ax_yaw, t, np.degrees(run["yaw_err"]), theme, "estimate",
                "Yaw error (estimate vs truth)", "angle (deg)", zero_line=True)

    if has_nis:
        ax_nis = fig.add_subplot(grid[2, 1], sharex=ax_pos)
        style_axes(ax_nis, theme)
        draw_series(ax_nis, t, run["nis"], theme, "estimate", "NIS of the EKF updates", "NIS",
                    sparse=True)
        ax_nis.set_ylim(bottom=0.0)

    final_pos = run["pos_err"][-1]
    final_yaw = math.degrees(run["yaw_err"][-1])
    fig.suptitle(title, x=0.06, ha="left", fontsize=13, color=theme["ink"])
    fig.text(0.06, 0.905, f"final estimate error: {final_pos * 100:.1f} cm position, "
             f"{final_yaw:+.2f} deg yaw   |   duration {t[-1]:.1f} s",
             fontsize=9.5, color=theme["ink_secondary"])
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("run_dir", type=Path, help="directory with run.csv and scene.json")
    parser.add_argument("-o", "--out", type=Path, help="output image (default: RUN_DIR/plot.png)")
    parser.add_argument("--dark", action="store_true", help="dark theme")
    parser.add_argument("--show", action="store_true", help="open a window instead of only saving")
    args = parser.parse_args()

    if not args.show:
        matplotlib.use("Agg")

    run_file = args.run_dir / "run.csv"
    if not run_file.exists():
        sys.exit(f"{run_file} not found")
    run = load_run(run_file)
    for needed in ("t_s", "truth_x", "truth_y", "truth_yaw", "est_x", "est_y", "pos_err", "yaw_err"):
        if needed not in run:
            sys.exit(f"{run_file}: missing column '{needed}'")
    scene = load_scene(args.run_dir / "scene.json")

    theme = THEMES["dark" if args.dark else "light"]
    title = scene.get("name") or args.run_dir.resolve().name
    fig = make_figure(run, scene, theme, title)

    out = args.out or args.run_dir / "plot.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    print(f"wrote {out}")
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
