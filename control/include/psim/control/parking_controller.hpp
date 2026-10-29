#pragma once

#include <cstdint>
#include <vector>

#include "psim/control/lateral_controller.hpp"
#include "psim/control/longitudinal_controller.hpp"
#include "psim/control/maneuver_fsm.hpp"
#include "psim/control/path_cursor.hpp"
#include "psim/msgs/interfaces.hpp"

namespace psim {

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct ControlParams {
  LateralParams lateral;
  LongitudinalParams longitudinal;
  ManeuverParams maneuver;

  // Which ultrasonic sensors look in the driving direction (PLAN.md layout: 0-3 front, 4-7 rear).
  std::vector<std::uint8_t> front_sensors{0, 1, 2, 3};
  std::vector<std::uint8_t> rear_sensors{4, 5, 6, 7};
  SimTime max_echo_age{300'000'000};  // older echoes are not used for the abort decision
  // A real obstacle returns a low echo on message after message, an outlier does not: a sensor
  // counts only after this many consecutive low echoes (a missing echo neither counts nor breaks
  // the series). A burst of random outliers (S4: 60 % of the readings of a dozen sensors for 1.5 s)
  // gives three low readings in a row from one sensor in 5-10 % of the runs, four in none of 40.
  int echo_confirmations{4};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

// The ECU's controller for a parking maneuver: follows a path with pure pursuit and a PI speed
// controller, one segment at a time, under the maneuver state machine. It works on the estimate it
// is given and knows nothing else: feeding it the truth instead (scenario S2) is the caller's
// decision, and impossible to do from in here.
class ParkingController final : public IController {
 public:
  explicit ParkingController(const ControlParams& params = {});

  // A path that differs from the previous one restarts the maneuver. `latest_uss` holds the newest
  // message of each ultrasonic sensor.
  [[nodiscard]] VehicleCmd compute(SimTime now, const EstimatedState& estimate, const Path& path,
                                   gsl::span<const UssMsg> latest_uss) override;

  [[nodiscard]] ManeuverState state() const { return fsm_.state(); }
  [[nodiscard]] const PathCursor& cursor() const { return cursor_; }

 private:
  [[nodiscard]] bool is_new_path(const Path& path);
  // Counts the low echoes of every sensor, one per new message.
  void track_echoes(SimTime now, double speed, gsl::span<const UssMsg> latest_uss);
  // The nearest confirmed low echo of the sensors that look in the driving direction.
  [[nodiscard]] std::optional<double> range_ahead(SimTime now, Direction direction) const;

  ControlParams params_;
  PathCursor cursor_;
  LateralController lateral_;
  LongitudinalController longitudinal_;
  ManeuverFsm fsm_;

  struct EchoTrack {
    std::optional<SimTime> last_stamp;  // of the newest message seen
    int low_streak{0};                  // consecutive low echoes
    SimTime last_low_stamp{};           // stamp of the latest low echo
    double last_low_range{0.0};
  };
  std::vector<EchoTrack> echo_tracks_;  // indexed by sensor id

  // What identifies the path being driven (size and the pose and speed of its first and last
  // point).
  struct Signature {
    std::size_t size{0};
    PathPoint first;
    PathPoint last;
  };
  Signature signature_;
  bool has_signature_{false};
};

}  // namespace psim
