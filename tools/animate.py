#!/usr/bin/env python3
"""Animate a closed-loop run (run.csv and scene.json of one RUN_DIR) as a GIF for the README.

Usage:
    python3 tools/animate.py RUN_DIR [-o OUT.gif] [--speed 2] [--fps 15] [--until T_S]

Left: top-down replay with the obstacles, the slot, the planned path, the car at its true pose (filled)
and at the estimated pose (dashed outline), and both trails. Right: speed, steering angle and the
estimate's position error against time, with a cursor at the replayed time. The maneuver is replayed
up to the last moment the truth moves (plus a second), `--until` overrides that. Light theme only.

The README animation (docs/img/s4_degraded_replay.gif):

    build/linux-gcc-debug/app/sim_main scenarios/S4_perpendicular_degraded.json --estimator ekf
    python3 tools/animate.py out/S4_perpendicular_degraded/ekf -o docs/img/s4_degraded_replay.gif --speed 2.5 --fps 12 \\
        --title "S4 perpendicular parking, degraded sensors" \\
        --subtitle "EKF + pure pursuit on 3 % wheel scale error, 0.01 rad/s gyro bias, 10 % ultrasonic dropouts, an outlier burst at 14 s"

Colors and the dash pattern of the estimate are those of plot.py (truth blue, estimate orange dashed).
Needs matplotlib, numpy and Pillow.
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  (after the backend is chosen)
import numpy as np  # noqa: E402

import plot  # noqa: E402

FIGURE_SIZE = (9.0, 5.4)  # inches, at 100 dpi the pixel size of the GIF
DPI = 100
MOVING_SPEED = 0.02       # m/s, the replay ends one second after the truth last moves faster than this
TAIL_S = 1.0
FINAL_HOLD_S = 1.5        # the last frame stays this long
PALETTE_COLORS = 96


def replay_end(data, until):
    if until is not None:
        return until
    moving = np.nonzero(np.abs(data["truth_v"]) > MOVING_SPEED)[0]
    last = data["t_s"][moving[-1]] if moving.size else data["t_s"][-1]
    return min(last + TAIL_S, data["t_s"][-1])


def build_figure(data, scene, theme, title, subtitle, t_end):
    from matplotlib.patches import Polygon as PolygonPatch

    fig = plt.figure(figsize=FIGURE_SIZE, dpi=DPI, facecolor=theme["surface"])
    grid = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.0], left=0.07, right=0.97, top=0.85, bottom=0.1,
                            wspace=0.22, hspace=0.75)
    fig.text(0.07, 0.955, title, fontsize=12, color=theme["ink"], ha="left", va="center")
    if subtitle:
        fig.text(0.07, 0.915, subtitle, fontsize=9, color=theme["ink_secondary"], ha="left", va="center")

    ax = fig.add_subplot(grid[:, 0])
    plot.style_axes(ax, theme)
    for obstacle in scene.get("obstacles", []):
        ax.add_patch(PolygonPatch(obstacle["vertices"], closed=True, facecolor=theme["obstacle_fill"],
                                  edgecolor=theme["obstacle_edge"], linewidth=0.8, zorder=1))
    slot = scene.get("slot")
    if slot:
        ax.add_patch(PolygonPatch(slot["vertices"], closed=True, fill=False, edgecolor=theme["ink_secondary"],
                                  linewidth=1.0, linestyle=(0, (4, 3)), zorder=2))
    path = scene.get("path", [])
    ax.plot([p[0] for p in path], [p[1] for p in path], color=theme["ink_secondary"], linewidth=1.0,
            linestyle=(0, (1, 2)), label="planned path", zorder=3)
    truth_trail, = ax.plot([], [], color=theme["truth"], linewidth=plot.LINE_WIDTH, label="truth", zorder=4)
    est_trail, = ax.plot([], [], color=theme["estimates"][0], linewidth=plot.LINE_WIDTH,
                         linestyle=plot.ESTIMATE_DASHES[0], label="EKF estimate", zorder=5)
    footprint = scene["car"]
    t_all = data["t_s"]
    truth_car = PolygonPatch(footprint, closed=True, facecolor=theme["truth"], alpha=0.22,
                             edgecolor=theme["truth"], linewidth=1.2, zorder=6)
    truth_outline = PolygonPatch(footprint, closed=True, fill=False, edgecolor=theme["truth"],
                                 linewidth=1.2, zorder=7)
    est_car = PolygonPatch(footprint, closed=True, fill=False, edgecolor=theme["estimates"][0],
                           linewidth=1.2, linestyle=(0, (3, 2)), zorder=8)
    for patch in (truth_car, truth_outline, est_car):
        ax.add_patch(patch)
    clock = ax.text(0.03, 0.97, "", transform=ax.transAxes, fontsize=10, color=theme["ink"], va="top",
                    family="monospace")
    # Limits hold the obstacles and the footprint at every pose (the car swings out beyond the rear axle path).
    corners = [corner for index in range(0, len(t_all), 10)
               for corner in plot.car_in_world(footprint, data["truth_x"][index], data["truth_y"][index],
                                               data["truth_yaw"][index])]
    xs = [v[0] for o in scene.get("obstacles", []) for v in o["vertices"]] + [c[0] for c in corners]
    ys = [v[1] for o in scene.get("obstacles", []) for v in o["vertices"]] + [c[1] for c in corners]
    margin = 0.6
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    plot.style_legend(ax.legend(loc="center right", frameon=False, fontsize=9), theme)

    t = data["t_s"]
    panels = [
        ("Speed (negative: reversing)", "m/s", data["truth_v"], theme["truth"]),
        ("Steering angle", "deg", np.degrees(data["truth_steer"]), theme["truth"]),
        ("Estimate position error", "cm", 100.0 * data["pos_err"], theme["estimates"][0]),
    ]
    cursors = []
    for row, (name, unit, values, color) in enumerate(panels):
        sub = fig.add_subplot(grid[row, 1])
        plot.style_axes(sub, theme)
        sub.plot(t, values, color=color, linewidth=1.4)
        sub.axhline(0.0, color=theme["ink_secondary"], linewidth=0.6)
        sub.set_title(name, loc="left", fontsize=10, color=theme["ink"])
        sub.set_ylabel(unit)
        sub.set_xlim(t[0], t_end)
        if row == len(panels) - 1:
            sub.set_xlabel("time (s)")
        else:
            sub.tick_params(labelbottom=False)
        cursors.append(sub.axvline(0.0, color=theme["ink"], linewidth=1.0))

    def update(index):
        truth_trail.set_data(data["truth_x"][: index + 1], data["truth_y"][: index + 1])
        est_trail.set_data(data["est_x"][: index + 1], data["est_y"][: index + 1])
        pose = (data["truth_x"][index], data["truth_y"][index], data["truth_yaw"][index])
        truth_pose = plot.car_in_world(footprint, *pose)
        truth_car.set_xy(truth_pose)
        truth_outline.set_xy(truth_pose)
        est_car.set_xy(plot.car_in_world(footprint, data["est_x"][index], data["est_y"][index],
                                         data["est_yaw"][index]))
        clock.set_text(f"t = {t[index]:5.1f} s\nv = {data['truth_v'][index]:+5.2f} m/s")
        for cursor in cursors:
            cursor.set_xdata([t[index], t[index]])

    return fig, update


def to_image(fig):
    from PIL import Image

    fig.canvas.draw()
    return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())


def write_gif(frames, path, frame_ms):
    from PIL import Image

    # One palette for all frames (taken from the last one, which holds every color of the drawing):
    # per-frame palettes make the lines flicker and the file larger.
    palette = frames[-1].quantize(colors=PALETTE_COLORS, method=Image.Quantize.MEDIANCUT,
                                  dither=Image.Dither.NONE)
    indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    durations = [frame_ms] * (len(indexed) - 1) + [int(FINAL_HOLD_S * 1000)]
    path.parent.mkdir(parents=True, exist_ok=True)
    indexed[0].save(path, save_all=True, append_images=indexed[1:], duration=durations, loop=0,
                    optimize=False, disposal=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("-o", "--output", type=Path, help="default: RUN_DIR/replay.gif")
    parser.add_argument("--speed", type=float, default=2.0, help="replay speed relative to sim time")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--until", type=float, help="end of the replay in sim seconds")
    parser.add_argument("--title", help="default: the scenario name of scene.json")
    parser.add_argument("--subtitle", default="", help="second line under the title")
    args = parser.parse_args()

    data = plot.load_run(args.run_dir / "run.csv")
    scene = plot.load_scene(args.run_dir / "scene.json")
    if "car" not in scene:
        sys.exit(f"{args.run_dir}/scene.json has no car footprint; rerun sim_main")
    end = replay_end(data, args.until)
    title = args.title or f"{scene.get('name', args.run_dir.name)}: EKF, closed loop"

    fig, update = build_figure(data, scene, plot.THEMES["light"], title, args.subtitle, end)
    times = np.arange(0.0, end, args.speed / args.fps)
    indices = np.searchsorted(data["t_s"], times)
    indices = np.minimum(indices, len(data["t_s"]) - 1)
    frames = []
    for index in list(indices) + [int(np.searchsorted(data["t_s"], end)) - 1]:
        update(int(index))
        frames.append(to_image(fig))
    output = args.output or args.run_dir / "replay.gif"
    write_gif(frames, output, round(1000.0 / args.fps))
    print(f"{output}: {len(frames)} frames, {output.stat().st_size / 1e6:.1f} MB, "
          f"{end:.1f} s of sim time at {args.speed:g}x")


if __name__ == "__main__":
    main()
