"""Hand-calculation model behind the side slip deck (docs/side_slip_kinematic_vs_dynamic.pptx).

Single source of the numbers on the slides and in the figures. These are closed-form steady-state
results for a linear single-track model with ASSUMED typical sedan parameters; they are not
measured on the simulator (the dynamic model is not built yet) and the deck says so.

Conventions: left positive; a = CG to front axle, b = CG to rear axle, L = a + b; R = radius of
the rear axle's path; alpha > 0 = lateral tire force to the left.
"""

import math
from dataclasses import dataclass

G = 9.81


@dataclass(frozen=True)
class Params:
    m: float = 1500.0     # kg
    iz: float = 2500.0    # kg m^2 (not used by the steady-state numbers)
    L: float = 2.7        # m, wheelbase (the simulator's VehicleParams)
    a: float = 1.2        # m, CG to front axle
    b: float = 1.5        # m, CG to rear axle
    cf: float = 80e3      # N/rad, front axle cornering stiffness
    cr: float = 90e3      # N/rad, rear axle cornering stiffness
    mu: float = 0.9       # dry asphalt
    steer_model_sigma: float = 0.003  # rad/s, EkfParams::steer_model_sigma
    dt: float = 0.01      # s, simulation tick


P = Params()


def understeer_gradient(p: Params = P) -> float:
    """K in rad per m/s^2: the extra steering angle per unit of lateral acceleration."""
    return p.m / p.L * (p.b / p.cf - p.a / p.cr)


def slip_front(ay: float, p: Params = P) -> float:
    return p.m * ay * p.b / (p.L * p.cf)


def slip_rear(ay: float, p: Params = P) -> float:
    return p.m * ay * p.a / (p.L * p.cr)


def beta_cg_kinematic(delta: float, p: Params = P) -> float:
    """Geometric body side slip of the kinematic bicycle at the CG (no tire slip involved)."""
    return math.atan(p.b * math.tan(delta) / p.L)


def yaw_rate_factor(v: float, p: Params = P) -> float:
    """r_dynamic / r_kinematic at the same steering angle: 1 / (1 + K v^2 / L)."""
    return 1.0 / (1.0 + understeer_gradient(p) * v * v / p.L)


def yaw_rate_error(v: float, radius: float, p: Params = P) -> float:
    """|r_kinematic - r_dynamic| in rad/s for the same steering angle on a turn of this radius."""
    r_kin = v / radius
    return r_kin * (1.0 - yaw_rate_factor(v, p))


def lateral_mode_rate(v: float, p: Params = P) -> float:
    """Magnitude of the fast lateral eigenvalue of slip-based tire forces, 1/s."""
    return (p.cf + p.cr) / (p.m * v)


RK4_REAL_AXIS_LIMIT = 2.785  # |lambda| dt must stay below this for explicit RK4


def rk4_min_speed(dt: float, p: Params = P) -> float:
    """Speed below which RK4 with step dt is unstable on the lateral mode."""
    return (p.cf + p.cr) / (p.m * RK4_REAL_AXIS_LIMIT / dt)


@dataclass(frozen=True)
class Case:
    name: str
    v: float       # m/s
    radius: float  # m, rear axle path

    @property
    def ay(self) -> float:
        return self.v * self.v / self.radius


PARKING = Case("parking", 1.0, 5.3)
DRIVE_BY = Case("drive-by", 5.0, 8.0)


if __name__ == "__main__":
    k = understeer_gradient()
    print(f"K = {k:.5f} rad/(m/s^2)")
    for case in (PARKING, DRIVE_BY):
        ar, af = slip_rear(case.ay), slip_front(case.ay)
        print(f"{case.name}: ay={case.ay:.3f}  alpha_r={math.degrees(ar):.3f} deg  "
              f"alpha_f={math.degrees(af):.3f} deg  K*ay={math.degrees(k * case.ay):.3f} deg  "
              f"crab over 10 m = {100 * 10 * ar:.2f} cm, over 20 m = {20 * ar:.2f} m  "
              f"yaw-rate factor {yaw_rate_factor(case.v):.4f}  "
              f"yaw-rate error {yaw_rate_error(case.v, case.radius):.4f} rad/s")
    print(f"beta_cg(0.3 rad) = {math.degrees(beta_cg_kinematic(0.3)):.2f} deg")
    print(f"mu g = {0.9 * G:.2f} m/s^2")
    for dt in (0.01, 0.005, 0.002, 0.001):
        print(f"RK4 dt={dt}: unstable below v = {rk4_min_speed(dt):.2f} m/s")
    print(f"lateral mode rate: 1 m/s {lateral_mode_rate(1.0):.0f}, 0.1 m/s {lateral_mode_rate(0.1):.0f} 1/s")
    # speed where the yaw-rate error reaches steer_model_sigma on R = 8 m
    v = 0.1
    while yaw_rate_error(v, 8.0) < P.steer_model_sigma:
        v += 0.01
    print(f"yaw-rate error exceeds steer_model_sigma at v = {v:.2f} m/s (R = 8 m)")
