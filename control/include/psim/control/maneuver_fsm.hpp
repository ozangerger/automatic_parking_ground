#pragma once

#include <optional>

#include "psim/msgs/path.hpp"
#include "psim/msgs/time.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

enum class ManeuverState : std::uint8_t {
  Idle,       // no path
  Approach,   // driving a forward segment
  ReverseIn,  // driving a reverse segment
  Done,       // the last segment is finished, the vehicle is parked
  Abort       // an obstacle was too close: braked to a standstill, parked, waiting for a new path
};

[[nodiscard]] const char* to_string(ManeuverState state);

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ManeuverParams {
  // An echo in the driving direction aborts when it is closer than this range at a standstill. A
  // moving vehicle needs more: the confirmation, the command delay and the braking distance
  // also pass before it stands, so the range grows with the speed (see abort_range_at).
  double abort_range{0.25};         // m
  double abort_reaction_time{0.3};  // s: confirmation, bus and actuator delay, sensor age
  double abort_decel{2.0};          // m/s^2: what Park brakes with (the vehicle's limit)
  SimTime abort_confirm{
      120'000'000};            // the low range must persist this long (one outlier must not abort)
  double end_tolerance{0.05};  // m: a segment is finished within this distance of its end ...
  double standstill_speed{0.05};            // m/s ... and below this speed
  SimTime standstill_confirm{200'000'000};  // ... for this long
  SimTime gear_dwell{
      600'000'000};  // after switching to the next segment: steer and engage the gear, then go
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

struct ManeuverInput {
  SimTime now{};
  bool has_path{false};
  bool new_path{false};                     // a different path than the previous call: start over
  Direction direction{Direction::Forward};  // of the current segment
  bool last_segment{true};
  double distance_to_go{0.0};         // m, to the end of the current segment
  double speed{0.0};                  // m/s, magnitude
  std::optional<double> range_ahead;  // nearest recent echo in the driving direction
};

struct ManeuverOutput {
  ManeuverState state{ManeuverState::Idle};
  Gear gear{Gear::Park};
  bool steer{false};            // let the lateral controller steer
  bool drive{false};            // let the longitudinal controller accelerate
  bool advance_segment{false};  // the cursor must move to the next segment now
};

// The maneuver as a sequence of path segments driven one after the other, stopping at the end of
// each. The gear changes only at a standstill: the next segment is started (steering and gear),
// and the vehicle waits for the dwell time before moving. An obstacle closer than the abort range
// in the driving direction (for long enough) ends the maneuver; a new path starts it again.
// ("Correct" of PLAN.md section 7.8 is optional and needs the planner; not part of this step.)
class ManeuverFsm {
 public:
  explicit ManeuverFsm(const ManeuverParams& params = {});

  [[nodiscard]] ManeuverState state() const { return state_; }
  // The echo range below which a vehicle driving at `speed` aborts.
  [[nodiscard]] double abort_range_at(double speed) const;

  [[nodiscard]] ManeuverOutput update(const ManeuverInput& in);

 private:
  [[nodiscard]] static ManeuverState state_for(Direction direction) {
    return direction == Direction::Forward ? ManeuverState::Approach : ManeuverState::ReverseIn;
  }
  [[nodiscard]] static Gear gear_for(Direction direction) {
    return direction == Direction::Forward ? Gear::Drive : Gear::Reverse;
  }

  ManeuverParams params_;
  ManeuverState state_{ManeuverState::Idle};
  std::optional<SimTime> low_range_since_;
  std::optional<SimTime> standstill_since_;
  std::optional<SimTime> resume_at_;  // the vehicle may move again from this time on
};

}  // namespace psim
