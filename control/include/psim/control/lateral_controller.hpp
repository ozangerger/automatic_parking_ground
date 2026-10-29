#pragma once

#include <cstdint>
#include <optional>

#include "psim/control/path_cursor.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

// The steering laws the ECU can use to follow a path (see the controller of the same name).
enum class LateralKind : std::uint8_t { PurePursuit, Stanley, Kanayama };

[[nodiscard]] const char* to_string(LateralKind kind);

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
// What every steering law shares: the vehicle and what the steering actuator can do.
struct LateralParams {
  double wheelbase{2.7};         // m
  double max_steer{0.55};        // rad
  double steer_rate_limit{0.5};  // rad/s: what the steering actuator can do
  // Close to the end of a segment the law gets erratic (PLAN.md section 14: the target point is
  // the end point itself, the path direction there is undefined): the steering is held for the
  // last stretch.
  double freeze_distance{0.3};  // m
  // The control period assumed for the first call, when no earlier call gives the time step.
  SimTime nominal_period{20'000'000};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// Steering angle command for following the current segment of a path. The base class owns what
// all laws share (the wheel angle limit, the rate limit between calls, holding the steering over
// the last stretch); a law only says which angle it wants.
class LateralController {
 public:
  explicit LateralController(const LateralParams& params);
  virtual ~LateralController() = default;
  LateralController(const LateralController&) = delete;
  LateralController& operator=(const LateralController&) = delete;
  LateralController(LateralController&&) = delete;
  LateralController& operator=(LateralController&&) = delete;

  // Forgets the previous steering command (a new path starts from `steer`).
  void reset(double steer = 0.0);

  // The steering angle command for the vehicle at (x, y, yaw), rear axle reference. `speed`
  // (magnitude) is the speed the vehicle is driving or about to drive; the direction comes from the
  // cursor. Limited to the wheel angle and to the rate the actuator can follow between calls.
  [[nodiscard]] double steer(const PathCursor& cursor, double x, double y, double yaw, double speed,
                             SimTime now);

  [[nodiscard]] double last_steer() const { return last_steer_; }

 protected:
  [[nodiscard]] const LateralParams& params() const { return params_; }

  // The angle the law wants, before the limits; nullopt keeps the previous command.
  [[nodiscard]] virtual std::optional<double> wanted_steer(const PathCursor& cursor, double x,
                                                           double y, double yaw,
                                                           double speed) const = 0;

 private:
  LateralParams params_;
  double last_steer_{0.0};
  SimTime last_time_{};
  bool has_last_time_{false};
};

}  // namespace psim
