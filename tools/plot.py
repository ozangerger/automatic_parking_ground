#!/usr/bin/env python3
"""Plot simulation runs written by CsvSink and write_scene.

Usage:
    python3 tools/plot.py RUN_DIR [RUN_DIR2 [RUN_DIR3]] [--label A --label B] [-o OUT.png] [--dark] [--show]

Each RUN_DIR holds run.csv (one row per tick, columns as documented in csv_sink.hpp) and,
optionally, scene.json (obstacles, slot, car footprint, planned path). With two run directories of the same
scenario (same seed, so the same truth) the estimates are overlaid, for example:

    sim_main scenarios/S1_constant_steering.json --estimator dead_reckoning
    sim_main scenarios/S1_constant_steering.json --estimator ekf
    python3 tools/plot.py out/S1_constant_steering/dead_reckoning out/S1_constant_steering/ekf

Panels: top-down trajectory of the truth and the estimate(s), position error and yaw error
against time, and NIS against time when run.csv has an `nis` column. When scene.json has a planned
path (closed-loop scenarios) three more panels follow: the distance of the truth to the path
(cross-track error), the steering angle (commanded and actual) and the speed. The controller works
on its estimate, so two runs of a closed-loop scenario drive different trajectories.

Three runs can be compared in a closed-loop scenario, for example the three steering laws:

    for c in pure_pursuit stanley kanayama; do sim_main scenarios/S3_perpendicular_estimate.json --controller $c; done
    python3 tools/plot.py out/S3_perpendicular_estimate/ekf out/S3_perpendicular_estimate/ekf_stanley \\
        out/S3_perpendicular_estimate/ekf_kanayama --label "pure pursuit" --label Stanley --label Kanayama

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

# Categorical slots 1-3 of the reference palette, validated for light and dark surfaces
# (scripts/validate_palette.py --pairs all). Aqua is below 3:1 contrast on the light surface, so
# every series is also told apart by its dash pattern and carries a label (legend and end label).
# The truth is slot 1 (blue); the estimates of two runs are slots 2 and 3. In a closed-loop
# comparison of three runs each run has its own truth and slot 1 is the third run's color.
THEMES = {
    "light": {
        "surface": "#fcfcfb",
        "ink": "#0b0b0b",
        "ink_secondary": "#52514e",
        "grid": "#e4e3df",
        "obstacle_fill": "#d6d5d0",
        "obstacle_edge": "#9a9992",
        "truth": "#2a78d6",
        "estimates": ["#eb6834", "#1baf7a", "#2a78d6"],
    },
    "dark": {
        "surface": "#1a1a19",
        "ink": "#f0efec",
        "ink_secondary": "#c3c2b7",
        "grid": "#33322f",
        "obstacle_fill": "#3a3936",
        "obstacle_edge": "#6e6d67",
        "truth": "#3987e5",
        "estimates": ["#d95926", "#199e70", "#3987e5"],
    },
}

# Dash patterns of the estimate lines, in the order of the run directories.
ESTIMATE_DASHES = [(0, (5, 2)), (0, (1, 1.5, 4, 1.5)), (0, (3, 1.5, 1, 1.5, 1, 1.5))]

LINE_WIDTH = 1.8
MARKER_SIZE = 8
MAX_RUNS = len(ESTIMATE_DASHES)


def load_run(path):
    """Returns {column: np.ndarray of float}; empty cells become NaN, text columns are dropped."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
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


def style_legend(legend, theme):
    for text in legend.get_texts():
        text.set_color(theme["ink_secondary"])


def draw_trajectory(ax, runs, scene, theme):
    from matplotlib.patches import Polygon as PolygonPatch

    for obstacle in scene.get("obstacles", []):
        ax.add_patch(PolygonPatch(obstacle["vertices"], closed=True, facecolor=theme["obstacle_fill"],
                                  edgecolor=theme["obstacle_edge"], linewidth=0.8, zorder=1))
    slot = scene.get("slot")
    if slot:
        ax.add_patch(PolygonPatch(slot["vertices"], closed=True, fill=False,
                                  edgecolor=theme["ink_secondary"], linewidth=1.0, linestyle=(0, (4, 3)),
                                  zorder=2))

    path = scene.get("path")
    if path:
        # The path the controller was given: neutral ink, thin, so that it stays a reference line.
        ax.plot([point[0] for point in path], [point[1] for point in path],
                color=theme["ink_secondary"], linewidth=1.0, linestyle=(0, (1, 2)),
                label="planned path", zorder=3)

    # In a closed-loop comparison every run drives its own trajectory (the controller works on the
    # estimate), so each run gets its own truth line, in the run's color; the estimate is the thin
    # dashed line beside it. Otherwise (open loop, or one run) all runs share one truth.
    separate_truth = bool(path) and len(runs) > 1
    footprint = scene.get("car")

    def mark_pose(index, name, data, color):
        x, y, yaw = data["truth_x"][index], data["truth_y"][index], data["truth_yaw"][index]
        ax.plot([x], [y], marker="o", markersize=MARKER_SIZE, color=color,
                markeredgecolor=theme["surface"], markeredgewidth=2, linestyle="none", zorder=7)
        if footprint:
            ax.add_patch(PolygonPatch(car_in_world(footprint, x, y, yaw), closed=True, fill=False,
                                      edgecolor=color if separate_truth else theme["ink_secondary"],
                                      linewidth=0.9, zorder=3))
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(8, 8), fontsize=9,
                    color=theme["ink_secondary"])

    if separate_truth:
        for index, run in enumerate(runs):
            color = theme["estimates"][index]
            data = run["data"]
            ax.plot(data["truth_x"], data["truth_y"], color=color, linewidth=LINE_WIDTH,
                    label=f"{run['label']} truth", zorder=4 + index)
            ax.plot(data["est_x"], data["est_y"], color=color, linewidth=1.0,
                    linestyle=ESTIMATE_DASHES[index], label=f"{run['label']} estimate", zorder=5 + index)
            mark_pose(-1, "end" if index == 0 else "", data, color)
        # With three runs the blue is a run's color: the start is marked in neutral ink.
        mark_pose(0, "start", runs[0]["data"], theme["ink"] if len(runs) > 2 else theme["truth"])
    else:
        truth = runs[0]["data"]
        ax.plot(truth["truth_x"], truth["truth_y"], color=theme["truth"], linewidth=LINE_WIDTH,
                label="truth", zorder=4)
        for index, run in enumerate(runs):
            ax.plot(run["data"]["est_x"], run["data"]["est_y"], color=theme["estimates"][index],
                    linewidth=LINE_WIDTH, linestyle=ESTIMATE_DASHES[index], label=run["label"],
                    zorder=5 + index)
        # Start and end of the truth path; the car outline shows heading and size at both poses.
        mark_pose(0, "start", truth, theme["truth"])
        mark_pose(-1, "end", truth, theme["truth"])

    ax.set_aspect("equal", adjustable="datalim")
    ax.autoscale_view()
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectory, top-down", loc="left", fontsize=11, color=theme["ink"])
    style_legend(ax.legend(loc="best", frameon=False, fontsize=9), theme)


def draw_series(ax, runs, column, theme, title, ylabel, scale=1.0, zero_line=False, sparse=False,
                legend_loc="upper left"):
    """One line per run. With sparse=True only the finite samples are drawn, as unconnected markers
    (events such as filter updates leave blank cells in between)."""
    ends = []
    plotted = 0
    for index, run in enumerate(runs):
        if column not in run["data"]:
            continue
        t = run["data"]["t_s"]
        values = run["data"][column] * scale
        color = theme["estimates"][index]
        if sparse:
            keep = np.isfinite(values)
            t, values = t[keep], values[keep]
            if not len(values):
                continue  # e.g. dead reckoning has no measurement updates
            ax.plot(t, values, color=color, linestyle="none", marker="o", markersize=3.0,
                    markeredgewidth=0, alpha=0.7, label=run["label"])
        else:
            ax.plot(t, values, color=color, linewidth=LINE_WIDTH, linestyle=ESTIMATE_DASHES[index],
                    label=run["label"])
        if len(values):
            ends.append((values[-1], t[-1], run["label"]))
            plotted += 1
    if zero_line:
        ax.axhline(0.0, color=theme["ink_secondary"], linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=11, color=theme["ink"])
    if plotted > 1:
        if legend_loc == "auto":
            # Lines leave zero towards the dominant drift: the legend goes to the empty corner on
            # the other side of the start.
            finals = [end[0] for end in ends]
            legend_loc = "upper left" if finals and max(finals, key=abs) > 0 else "lower left"
        style_legend(ax.legend(loc=legend_loc, frameon=False, fontsize=8.5), theme)
        label_line_ends(ax, ends, theme)


NIS_GATE = 6.635   # chi-square, 1 degree of freedom, 99 %: updates above it are rejected
NIS_AXIS_TOP = 12.0


def draw_nis_reference(ax, runs, theme):
    """The gate, a bounded axis, and how many values the bound cuts off (rejected outliers)."""
    ax.axhline(NIS_GATE, color=theme["ink_secondary"], linewidth=0.8, linestyle=(0, (4, 3)))
    ax.annotate("gate 6.6", (ax.get_xlim()[1], NIS_GATE), xytext=(6, 0), textcoords="offset points",
                fontsize=8.5, color=theme["ink_secondary"], va="center", annotation_clip=False)
    ax.set_ylim(0.0, NIS_AXIS_TOP)
    above = sum(int(np.sum(run["data"]["nis"][np.isfinite(run["data"]["nis"])] > NIS_AXIS_TOP))
                for run in runs if "nis" in run["data"])
    if above:
        ax.text(0.99, 0.95, f"{above} ticks above {NIS_AXIS_TOP:.0f} not shown (rejected outliers)",
                transform=ax.transAxes, ha="right", va="top", fontsize=8.5, color=theme["ink_secondary"])


def label_line_ends(ax, ends, theme):
    """Direct labels right of the line ends, nudged apart where they would collide."""
    if not ends:
        return
    low, high = ax.get_ylim()
    min_gap = 0.09 * (high - low)
    placed = []
    for value, t_end, label in sorted(ends):
        y = value if not placed else max(value, placed[-1] + min_gap)
        placed.append(y)
        ax.annotate(label, (t_end, y), xytext=(6, 0), textcoords="offset points", fontsize=8.5,
                    color=theme["ink_secondary"], va="center", ha="left", annotation_clip=False)


def distance_to_polyline(points, xs, ys):
    """Distance of each (x, y) to the polyline through `points` (a list of [x, y])."""
    path = np.asarray(points, dtype=float)
    if len(path) < 2:
        return np.hypot(xs - path[0, 0], ys - path[0, 1])
    a, b = path[:-1], path[1:]
    ab = b - a
    length_sq = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    best = np.full(len(xs), np.inf)
    for index, (x, y) in enumerate(zip(xs, ys)):
        t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / length_sq, 0.0, 1.0)
        best[index] = np.min(np.hypot(x - (a[:, 0] + t * ab[:, 0]), y - (a[:, 1] + t * ab[:, 1])))
    return best


def draw_steering(ax, runs, theme):
    """Commanded steering (thin) and the actual front wheel angle (dashed like the run)."""
    for index, run in enumerate(runs):
        data = run["data"]
        color = theme["estimates"][index]
        if "cmd_steer" in data:
            ax.plot(data["t_s"], np.degrees(data["cmd_steer"]), color=color, linewidth=0.9, alpha=0.6,
                    label=f"{run['label']} command")
        ax.plot(data["t_s"], np.degrees(data["truth_steer"]), color=color, linewidth=LINE_WIDTH,
                linestyle=ESTIMATE_DASHES[index], label=f"{run['label']} actual")
    ax.axhline(0.0, color=theme["ink_secondary"], linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("angle (deg)")
    ax.set_title("Steering angle: command and actual", loc="left", fontsize=11, color=theme["ink"])
    style_legend(ax.legend(loc="lower right", frameon=False, fontsize=8.5), theme)


def rms(values):
    values = values[np.isfinite(values)]
    return math.sqrt(float(np.mean(values ** 2))) if len(values) else math.nan


def summary_line(runs):
    parts = [f"{run['label']}: {run['data']['pos_err'][-1] * 100:.1f} cm, "
             f"{math.degrees(run['data']['yaw_err'][-1]):+.2f} deg" for run in runs]
    duration = runs[0]["data"]["t_s"][-1]
    return "final estimate error   " + "   |   ".join(parts) + f"   |   duration {duration:.1f} s"


def make_figure(runs, scene, theme, title):
    import matplotlib.pyplot as plt

    has_nis = any("nis" in run["data"] and np.isfinite(run["data"]["nis"]).any() for run in runs)
    path = scene.get("path")
    panels = 2 + (1 if has_nis else 0) + (3 if path else 0)

    fig = plt.figure(figsize=(12.5, 6.5 if panels <= 3 else 2.1 * panels + 1.0), facecolor=theme["surface"])
    top = 0.83 if panels <= 3 else 0.90
    grid = fig.add_gridspec(panels, 2, width_ratios=(1.5, 1.0), hspace=0.75 if path else 0.55,
                            wspace=0.34, left=0.06, right=0.90, top=top, bottom=0.06)

    ax_traj = fig.add_subplot(grid[:, 0])
    style_axes(ax_traj, theme)
    draw_trajectory(ax_traj, runs, scene, theme)

    row = 0
    ax_pos = fig.add_subplot(grid[row, 1])
    style_axes(ax_pos, theme)
    draw_series(ax_pos, runs, "pos_err", theme, "Position error (estimate vs truth)", "distance (m)")
    ax_pos.set_ylim(bottom=0.0)

    row += 1
    ax_yaw = fig.add_subplot(grid[row, 1], sharex=ax_pos)
    style_axes(ax_yaw, theme)
    draw_series(ax_yaw, runs, "yaw_err", theme, "Yaw error (estimate vs truth)", "angle (deg)",
                scale=180.0 / math.pi, zero_line=True, legend_loc="auto")

    if has_nis:
        row += 1
        ax_nis = fig.add_subplot(grid[row, 1], sharex=ax_pos)
        style_axes(ax_nis, theme)
        draw_series(ax_nis, runs, "nis", theme, "NIS of the filter updates (per tick)", "NIS",
                    sparse=True)
        draw_nis_reference(ax_nis, runs, theme)

    if path:
        for run in runs:
            data = run["data"]
            data["cross_track"] = distance_to_polyline(path, data["truth_x"], data["truth_y"])
        row += 1
        ax_ct = fig.add_subplot(grid[row, 1], sharex=ax_pos)
        style_axes(ax_ct, theme)
        draw_series(ax_ct, runs, "cross_track", theme, "Distance of the truth to the planned path",
                    "distance (m)", scale=1.0)
        ax_ct.set_ylim(bottom=0.0)

        row += 1
        ax_steer = fig.add_subplot(grid[row, 1], sharex=ax_pos)
        style_axes(ax_steer, theme)
        draw_steering(ax_steer, runs, theme)

        row += 1
        ax_speed = fig.add_subplot(grid[row, 1], sharex=ax_pos)
        style_axes(ax_speed, theme)
        draw_series(ax_speed, runs, "truth_v", theme, "Speed (negative: reversing)", "speed (m/s)",
                    zero_line=True, legend_loc="lower right")

    fig.suptitle(title, x=0.06, ha="left", fontsize=13, color=theme["ink"])
    fig.text(0.06, top + 0.025, summary_line(runs), fontsize=9.5, color=theme["ink_secondary"])
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("run_dirs", type=Path, nargs="+", help="one to three directories with run.csv")
    parser.add_argument("--label", action="append", default=[], help="legend label of a run (repeatable)")
    parser.add_argument("-o", "--out", type=Path, help="output image (default: first RUN_DIR/plot.png)")
    parser.add_argument("--dark", action="store_true", help="dark theme")
    parser.add_argument("--show", action="store_true", help="open a window instead of only saving")
    args = parser.parse_args()

    if len(args.run_dirs) > MAX_RUNS:
        sys.exit(f"at most {MAX_RUNS} runs can be compared in one plot")
    if len(args.label) > len(args.run_dirs):
        sys.exit("more --label options than run directories")
    if not args.show:
        matplotlib.use("Agg")

    runs = []
    for index, run_dir in enumerate(args.run_dirs):
        run_file = run_dir / "run.csv"
        if not run_file.exists():
            sys.exit(f"{run_file} not found")
        data = load_run(run_file)
        for needed in ("t_s", "truth_x", "truth_y", "truth_yaw", "est_x", "est_y", "pos_err", "yaw_err"):
            if needed not in data:
                sys.exit(f"{run_file}: missing column '{needed}'")
        default_label = "estimate" if len(args.run_dirs) == 1 else run_dir.resolve().name
        label = args.label[index] if index < len(args.label) else default_label
        runs.append({"label": label, "data": data})

    scene = load_scene(args.run_dirs[0] / "scene.json")
    if len(runs) > 2 and not scene.get("path"):
        sys.exit("three runs can only be compared in a closed-loop scenario (each run has its own "
                 "truth; in an open-loop plot the blue is the shared truth)")
    if len(runs) == 2 and not scene.get("path"):  # closed loop: the truth differs by design
        a, b = runs[0]["data"], runs[1]["data"]
        if len(a["t_s"]) != len(b["t_s"]) or np.max(np.abs(a["truth_x"] - b["truth_x"])) > 1e-6:
            print("warning: the two runs have different truth; compare runs of one scenario "
                  "with the same seed", file=sys.stderr)

    theme = THEMES["dark" if args.dark else "light"]
    title = scene.get("name") or args.run_dirs[0].resolve().parent.name
    fig = make_figure(runs, scene, theme, title)

    for run in runs:
        data = run["data"]
        print(f"{run['label']}: position RMSE {rms(data['pos_err']):.3f} m, "
              f"yaw RMSE {math.degrees(rms(data['yaw_err'])):.2f} deg")

    out = args.out or args.run_dirs[0] / "plot.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    print(f"wrote {out}")
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
