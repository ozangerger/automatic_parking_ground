#!/usr/bin/env python3
"""Figures of the side slip deck, written to out/slides/side_slip/ (git-ignored).

    ~/.venvs/psim-slides/bin/python tools/slides/side_slip_figures.py

Colors: categorical slots 1-3 of the reference palette (blue, orange, aqua), neutral ink for
reference lines; every series is labeled directly. The geometry diagrams are schematic: the angles
are exaggerated and say so.
"""

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Arc, FancyArrowPatch, Polygon, Rectangle  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import side_slip_model as model  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "out" / "slides" / "side_slip"

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, FILL = "#0b0b0b", "#52514e", "#e4e3df", "#d6d5d0"

plt.rcParams.update({
    "font.family": ["DejaVu Sans"],
    "font.size": 13,
    "axes.edgecolor": INK2,
    "axes.labelcolor": INK2,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "mathtext.fontset": "dejavusans",
})

P = model.P


def style(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.grid(True, color=GRID, linewidth=0.7)
    ax.set_axisbelow(True)


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    fig.savefig(path, dpi=200, facecolor="white", bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)
    print(f"wrote {path}")


def rot(points, angle, origin=(0.0, 0.0)):
    c, s = math.cos(angle), math.sin(angle)
    return [(origin[0] + c * x - s * y, origin[1] + s * x + c * y) for x, y in points]


def wheel(ax, center, heading, color=INK, length=0.75, width=0.28):
    corners = [(-length / 2, -width / 2), (length / 2, -width / 2),
               (length / 2, width / 2), (-length / 2, width / 2)]
    ax.add_patch(Polygon(rot(corners, heading, center), closed=True, facecolor=color,
                         edgecolor=color, zorder=4))


def car(ax, delta=0.0, with_wheels=True):
    """Top view, heading +x, rear axle at the origin (the simulator's vehicle frame)."""
    length, width, rear, front = 4.5, 1.8, 0.9, 0.9
    ax.add_patch(Rectangle((-rear, -width / 2), length, width, facecolor="#f3f2ee",
                           edgecolor=INK2, linewidth=1.3, zorder=2))
    if with_wheels:
        half = 0.8
        for sign in (-1, 1):
            wheel(ax, (0.0, sign * half), 0.0)
            wheel(ax, (P.L, sign * half), delta)
    ax.plot([0, 0], [-1.1, 1.1], color=INK2, linewidth=0.8, linestyle=(0, (3, 2)), zorder=3)
    ax.plot([P.L, P.L], [-1.1, 1.1], color=INK2, linewidth=0.8, linestyle=(0, (3, 2)), zorder=3)


def arrow(ax, start, angle, length, color, width=2.2, zorder=6):
    end = (start[0] + length * math.cos(angle), start[1] + length * math.sin(angle))
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=16, color=color,
                                 linewidth=width, zorder=zorder, shrinkA=0, shrinkB=0))
    return end


def angle_arc(ax, center, radius, start_deg, end_deg, color, label, label_radius, label_angle_deg):
    ax.add_patch(Arc(center, 2 * radius, 2 * radius, theta1=min(start_deg, end_deg),
                     theta2=max(start_deg, end_deg), color=color, linewidth=2.0, zorder=7))
    lx = center[0] + label_radius * math.cos(math.radians(label_angle_deg))
    ly = center[1] + label_radius * math.sin(math.radians(label_angle_deg))
    ax.text(lx, ly, label, color=color, fontsize=15, fontweight="bold", ha="center", va="center",
            zorder=8)


# ---- slide 3: the angles ------------------------------------------------------------------------

def fig_angles():
    # A left turn drawn with exaggerated slip (as on a slippery surface), geometry kept consistent:
    # the velocity of the rear axle, the CG and the front axle follow from yaw rate and sideslip.
    delta, alpha_f, alpha_r = map(math.radians, (26.0, 11.0, 7.0))
    r_over_v = (math.tan(delta - alpha_f) + math.tan(alpha_r)) / P.L  # yaw rate / speed
    vy_rear = -math.tan(alpha_r)                                       # per unit v_x
    beta = math.atan(vy_rear + P.b * r_over_v)
    front_velocity = delta - alpha_f
    cg = (P.b, 0.0)
    front = (P.L, 0.0)
    ref = dict(color=INK2, linewidth=1.1, linestyle=(0, (4, 3)), zorder=5)

    fig, axes = plt.subplots(1, 3, figsize=(13.0, 3.5))
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_axis_off()
        car(ax, delta)
        ax.set_xlim(-1.3, 8.2)
        ax.set_ylim(-2.0, 3.1)

    # rear tire slip: the rear wheel points along the body, the rear axle moves along -alpha_r
    ax = axes[0]
    ax.plot([0, 4.6], [0, 0], **ref)
    arrow(ax, (0.0, 0.0), -alpha_r, 4.6, ORANGE)
    angle_arc(ax, (0.0, 0.0), 3.0, -7.0, 0.0, ORANGE, r"$\alpha_r$", 3.45, 6.5)
    ax.text(4.8, 0.12, "rear wheel heading", color=INK2, fontsize=11, va="bottom", ha="left")
    ax.text(4.8, -0.95, "rear axle velocity", color=ORANGE, fontsize=11.5, va="top")
    ax.set_title("rear tire slip", fontsize=14, color=INK, loc="left")

    # body side slip: the velocity of the CG against the body x-axis
    ax = axes[1]
    ax.plot([cg[0], cg[0] + 4.6], [0, 0], **ref)
    arrow(ax, cg, beta, 4.6, BLUE)
    angle_arc(ax, cg, 3.0, 0.0, math.degrees(beta), BLUE, r"$\beta$", 3.6, 11.0)
    ax.plot(*cg, marker="o", markersize=7, color=INK, zorder=9)
    ax.text(cg[0], -0.45, "CG", color=INK, fontsize=11, ha="center", zorder=9)
    ax.text(cg[0] + 4.8, -0.2, "body x-axis", color=INK2, fontsize=11, va="top", ha="left")
    ax.text(cg[0] + 4.8, 0.75, "CG velocity", color=BLUE, fontsize=11.5, va="bottom")
    ax.set_title("body side slip", fontsize=14, color=INK, loc="left")

    # front tire slip: the front wheel points along delta, the front axle moves along delta - alpha_f
    ax = axes[2]
    ax.plot([front[0], front[0] + 4.6 * math.cos(delta)], [0, 4.6 * math.sin(delta)], **ref)
    arrow(ax, front, front_velocity, 4.6, AQUA)
    angle_arc(ax, front, 3.0, math.degrees(front_velocity), math.degrees(delta), AQUA,
              r"$\alpha_f$", 3.65, 19.5)
    ax.text(front[0] + 4.6 * math.cos(delta) + 0.1, 4.6 * math.sin(delta) + 0.05, "front wheel\nheading",
            color=INK2, fontsize=11, va="bottom", ha="left")
    ax.text(front[0] + 4.7, 1.0, "front axle\nvelocity", color=AQUA, fontsize=11.5, va="top")
    ax.set_title("front tire slip", fontsize=14, color=INK, loc="left")
    axes[0].text(-1.2, -1.95, "angles exaggerated (slippery surface)", color=INK2, fontsize=10.5,
                 style="italic")
    save(fig, "angles.png")


# ---- slide 5: kinematic bicycle -----------------------------------------------------------------

def fig_kinematic():
    delta = math.radians(22.0)
    radius = P.L / math.tan(delta)
    fig, ax = plt.subplots(figsize=(7.6, 5.4))
    ax.set_aspect("equal")
    ax.set_axis_off()
    car(ax, delta)
    icr = (0.0, radius)
    # the two perpendiculars meet at the instantaneous center of rotation
    ax.plot([0, 0], [0, radius], color=INK2, linewidth=1.0, linestyle=(0, (4, 3)), zorder=1)
    ax.plot([P.L, icr[0]], [0, icr[1]], color=INK2, linewidth=1.0, linestyle=(0, (4, 3)), zorder=1)
    ax.plot(*icr, marker="o", markersize=8, color=INK, zorder=9)
    ax.text(0.25, radius + 0.35, "center of rotation", color=INK, fontsize=12)
    theta = np.linspace(-0.55, 0.42, 60)
    ax.plot(icr[0] + radius * np.sin(theta), icr[1] - radius * np.cos(theta), color=BLUE,
            linewidth=2.0, linestyle=(0, (5, 2)), zorder=3)
    arrow(ax, (0.0, 0.0), 0.0, 2.6, BLUE)
    arrow(ax, (P.L, 0.0), delta, 2.6, AQUA)
    ax.text(3.9, -0.75, "rear axle: $v$ along the\nwheel, " r"$\alpha_r = 0$", color=BLUE, fontsize=12, va="center")
    ax.text(P.L + 2.75, 1.2, "front axle:\n" r"$\alpha_f = 0$", color=AQUA, fontsize=12.5, va="center")
    ax.add_patch(Arc((P.L, 0.0), 2.4, 2.4, theta1=0, theta2=22, color=INK2, linewidth=1.5, zorder=7))
    ax.text(-0.15, radius / 2, r"$R=L/\tan\delta$", color=INK, fontsize=14, ha="right",
            va="center")
    ax.annotate("", xy=(0.0, -1.35), xytext=(P.L, -1.35),
                arrowprops=dict(arrowstyle="<->", color=INK2, linewidth=1.2))
    ax.text(P.L / 2, -1.85, r"$L$", color=INK2, fontsize=14, ha="center")
    ax.text(P.L + 1.45, 0.2, r"$\delta$", color=INK2, fontsize=15)
    ax.set_xlim(-2.8, 9.4)
    ax.set_ylim(-2.4, radius + 1.0)
    save(fig, "kinematic.png")


# ---- slide 6: the geometric body side slip ----------------------------------------------------

def fig_beta_geometric():
    delta = np.linspace(0.0, P.L and 0.55, 100)
    beta = np.degrees([model.beta_cg_kinematic(d) for d in delta])
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    style(ax)
    deg = np.degrees(delta)
    ax.plot(deg, beta, color=BLUE, linewidth=2.2)
    ax.plot(deg, np.zeros_like(deg), color=ORANGE, linewidth=2.2, linestyle=(0, (5, 2)))
    ax.text(deg[-1] + 0.5, beta[-1], r"$\beta$ at the CG", color=BLUE, va="center", fontsize=13)
    ax.text(deg[-1] + 0.5, 0.0, r"$\alpha_f=\alpha_r=0$" "\n(no tire slip)", color=ORANGE,
            va="center", fontsize=13)
    d03 = math.degrees(0.3)
    b03 = math.degrees(model.beta_cg_kinematic(0.3))
    ax.plot([d03], [b03], marker="o", color=BLUE, markersize=7)
    ax.annotate(f"0.3 rad of steering:\n{b03:.1f}° body side slip, 0° tire slip", (d03, b03),
                xytext=(1.5, 14.0), fontsize=12, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, linewidth=0.8))
    ax.set_xlabel("front wheel angle δ (deg)")
    ax.set_ylabel("angle (deg)")
    ax.set_xlim(0, deg[-1] + 12)
    ax.set_ylim(-1.5, 20)
    save(fig, "beta_geometric.png")


# ---- slide 7: the slip loop -----------------------------------------------------------------

def fig_dynamic_loop():
    fig, ax = plt.subplots(figsize=(11.6, 3.7))
    ax.set_axis_off()
    ax.set_xlim(0, 11.6)
    ax.set_ylim(0, 3.7)

    def box(x, y, w, h, title, body, color):
        ax.add_patch(Rectangle((x, y), w, h, facecolor="white", edgecolor=color, linewidth=2.0,
                               zorder=2))
        ax.text(x + w / 2, y + h - 0.28, title, ha="center", va="center", fontsize=13.5,
                fontweight="bold", color=color, zorder=3)
        ax.text(x + w / 2, y + (h - 0.4) / 2, body, ha="center", va="center", fontsize=12,
                color=INK, zorder=3, linespacing=1.55)

    box(0.1, 0.9, 3.15, 2.0, "Slip angles",
        r"$\alpha_f=\delta-\mathrm{atan}\frac{v_y+a\,r}{v_x}$" "\n"
        r"$\alpha_r=-\mathrm{atan}\frac{v_y-b\,r}{v_x}$", BLUE)
    box(4.15, 0.9, 3.0, 2.0, "Tire forces",
        r"$F_y=\mathrm{clamp}(C\,\alpha,\ \pm\mu F_z)$" "\n" "linear, then saturated", ORANGE)
    box(8.05, 0.9, 3.45, 2.0, "Body (at the CG)",
        r"$m(\dot v_y+v_x r)=F_{yf}\cos\delta+F_{yr}$" "\n"
        r"$I_z\dot r=a\,F_{yf}\cos\delta-b\,F_{yr}$", AQUA)
    for x0, x1 in ((3.25, 4.15), (7.15, 8.05)):
        ax.add_patch(FancyArrowPatch((x0, 1.9), (x1, 1.9), arrowstyle="-|>", mutation_scale=18,
                                     color=INK2, linewidth=2.0))
    # feedback: the states v_y and r return to the slip angles
    ax.add_patch(FancyArrowPatch((9.8, 0.9), (9.8, 0.35), arrowstyle="-", color=INK2, linewidth=2.0))
    ax.plot([9.8, 1.7], [0.35, 0.35], color=INK2, linewidth=2.0)
    ax.add_patch(FancyArrowPatch((1.7, 0.35), (1.7, 0.9), arrowstyle="-|>", mutation_scale=18,
                                 color=INK2, linewidth=2.0))
    ax.text(5.75, 0.08, r"states $v_y$ and $r$ (and $v_x$) change the slip", ha="center",
            va="bottom", fontsize=12, color=INK2)
    ax.add_patch(FancyArrowPatch((1.7, 3.6), (1.7, 2.9), arrowstyle="-|>", mutation_scale=18,
                                 color=INK2, linewidth=2.0))
    ax.text(1.95, 3.45, r"steering angle $\delta$", fontsize=12, color=INK2, va="center")
    save(fig, "dynamic_loop.png")


# ---- slide 9: slip against lateral acceleration --------------------------------------------

def fig_alpha_vs_ay():
    ay = np.logspace(-1.3, math.log10(9.5), 200)
    af = np.degrees([model.slip_front(a) for a in ay])
    ar = np.degrees([model.slip_rear(a) for a in ay])
    limit = P.mu * model.G
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    style(ax)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.axvspan(0.05, 0.25, color=GRID, alpha=0.9, zorder=0)
    ax.text(0.113, 0.6, "parking\n(≤ 1 m/s,\nR ≥ 4.4 m)", fontsize=11.5, color=INK2, va="center",
            ha="center")
    ax.plot(ay, af, color=AQUA, linewidth=2.2, linestyle=(0, (1, 1.5, 4, 1.5)))
    ax.plot(ay, ar, color=ORANGE, linewidth=2.2)
    ax.text(0.31, model.slip_front(0.31) * 57.3 * 1.5, r"$\alpha_f$", color=AQUA, fontsize=14,
            fontweight="bold")
    ax.text(0.31, model.slip_rear(0.31) * 57.3 * 0.55, r"$\alpha_r$", color=ORANGE, fontsize=14,
            fontweight="bold")
    ax.axhline(5.0, color=INK2, linewidth=1.0, linestyle=(0, (4, 3)))
    ax.text(0.27, 5.6, "linear tire model valid up to about 5°", fontsize=11, color=INK2,
            va="bottom")
    ax.axvline(limit, color=INK2, linewidth=1.0, linestyle=(0, (4, 3)))
    ax.text(limit * 0.93, 0.0145, "μ·g\n(sat.)", fontsize=11, color=INK2, ha="right", va="bottom")
    d = model.DRIVE_BY
    ax.plot([d.ay], [math.degrees(model.slip_rear(d.ay))], marker="o", color=ORANGE, markersize=8,
            markeredgecolor="white", markeredgewidth=1.5, zorder=9)
    ax.annotate("5 m/s, R = 8 m:\n" r"$\alpha_r$ = 1.3°", (d.ay, math.degrees(model.slip_rear(d.ay))),
                xytext=(1.05, 0.045), fontsize=11.5, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, linewidth=0.8))
    p = model.PARKING
    ax.plot([p.ay], [math.degrees(model.slip_rear(p.ay))], marker="o", color=ORANGE, markersize=8,
            markeredgecolor="white", markeredgewidth=1.5, zorder=9)
    ax.annotate("1 m/s, R = 5.3 m:\n" r"$\alpha_r$ = 0.08°", (p.ay, math.degrees(model.slip_rear(p.ay))),
                xytext=(0.32, 0.011), fontsize=11.5, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, linewidth=0.8))
    ax.set_xlabel("lateral acceleration (m/s², steady turn)")
    ax.set_ylabel("tire slip angle (deg)")
    ax.set_xlim(0.05, 11)
    ax.set_ylim(0.01, 12)
    save(fig, "alpha_vs_ay.png")


# ---- slide 11: the stiffness at low speed ---------------------------------------------------

def fig_stiffness():
    v = np.logspace(-1.2, 0.6, 200)
    lam = np.array([model.lateral_mode_rate(x) for x in v])
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    style(ax)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.axvspan(0.1, 1.0, color=GRID, alpha=0.9, zorder=0)
    ax.text(0.31, 3000, "parking speeds", fontsize=11.5, color=INK2, ha="center")
    ax.plot(v, lam, color=ORANGE, linewidth=2.4)
    ax.text(0.068, 55, "(A) slip-based tire forces:\nrate (C$_f$+C$_r$)/(m·v)", color=ORANGE,
            fontsize=12, va="center")
    limit = model.RK4_REAL_AXIS_LIMIT / P.dt
    ax.axhline(limit, color=INK, linewidth=1.3, linestyle=(0, (4, 3)))
    ax.text(3.6, limit * 1.12, "RK4 limit at 10 ms", fontsize=11.5, color=INK, ha="right")
    ax.plot([model.rk4_min_speed(P.dt)], [limit], marker="o", color=INK, markersize=7,
            markeredgecolor="white", markeredgewidth=1.5, zorder=9)
    ax.annotate(f"unstable below\n{model.rk4_min_speed(P.dt):.2f} m/s", (model.rk4_min_speed(P.dt), limit),
                xytext=(0.62, 50), fontsize=11.5, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, linewidth=0.8))
    spring = math.sqrt(P.cf / 0.4 / P.m)  # tire deflection spring, relaxation length 0.4 m
    ax.axhline(spring, color=AQUA, linewidth=2.4)
    ax.text(3.6, spring * 1.15, "(B) tire deflection states: ≈ 12 rad/s at any speed", color=AQUA,
            fontsize=12, ha="right")
    ax.set_xlabel("speed |v_x| (m/s)")
    ax.set_ylabel("fastest mode (1/s)")
    ax.set_xlim(0.063, 4)
    ax.set_ylim(5, 5000)
    save(fig, "stiffness.png")


# ---- slide 12: what the ECU's kinematic yaw-rate model gets wrong --------------------------

def fig_ecu_error():
    v = np.linspace(0.0, 6.0, 200)
    err = np.array([model.yaw_rate_error(x, 8.0) for x in v])
    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    style(ax)
    ax.axvspan(0.0, 1.0, color=GRID, alpha=0.9, zorder=0)
    ax.text(0.5, 0.0215, "parking", fontsize=11.5, color=INK2, ha="center")
    ax.plot(v, err, color=BLUE, linewidth=2.4)
    ax.text(4.3, 0.0215, "gyro vs v·tan δ / L\non an R = 8 m turn", color=BLUE, fontsize=12,
            ha="right", va="center")
    ax.axhline(P.steer_model_sigma, color=ORANGE, linewidth=2.0, linestyle=(0, (5, 2)))
    ax.text(5.75, P.steer_model_sigma + 0.0006, "steer_model_sigma = 0.003 rad/s (EKF)",
            color=ORANGE, fontsize=12, ha="right", va="bottom")
    cross = 2.8
    ax.plot([cross], [P.steer_model_sigma], marker="o", color=BLUE, markersize=7,
            markeredgecolor="white", markeredgewidth=1.5, zorder=9)
    ax.annotate(f"model error exceeds\nthe EKF's assumption\nfrom {cross:.1f} m/s", (cross, P.steer_model_sigma),
                xytext=(1.25, 0.0115), fontsize=11.5, color=INK2,
                arrowprops=dict(arrowstyle="-", color=INK2, linewidth=0.8))
    ax.set_xlabel("speed (m/s)")
    ax.set_ylabel("yaw-rate model error (rad/s)")
    ax.set_xlim(0, 6)
    ax.set_ylim(0, 0.025)
    save(fig, "ecu_error.png")


def main():
    fig_angles()
    fig_kinematic()
    fig_beta_geometric()
    fig_dynamic_loop()
    fig_alpha_vs_ay()
    fig_stiffness()
    fig_ecu_error()


if __name__ == "__main__":
    main()
