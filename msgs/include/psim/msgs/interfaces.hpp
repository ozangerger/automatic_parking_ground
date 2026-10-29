#pragma once

#include <cstdint>
#include <gsl/span>
#include <vector>

#include "psim/msgs/estimate.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/path.hpp"
#include "psim/msgs/prior_map.hpp"
#include "psim/msgs/snapshot.hpp"
#include "psim/msgs/time.hpp"

namespace psim {

// Polymorphic bases: virtual destructor, no copying or moving (C.35, C.67).
// Implementations mark every override with `override` (C.128).

class IEstimator {
 public:
  virtual ~IEstimator() = default;
  IEstimator(const IEstimator&) = delete;
  IEstimator& operator=(const IEstimator&) = delete;
  IEstimator(IEstimator&&) = delete;
  IEstimator& operator=(IEstimator&&) = delete;

  virtual void on_message(const SensorMsg& msg) = 0;
  virtual void predict_to(SimTime t) = 0;
  [[nodiscard]] virtual EstimatedState estimate() const = 0;

  // Normalized innovation squared of the measurement updates since the last call, one value per
  // scalar update. Estimators without measurement updates return nothing.
  [[nodiscard]] virtual std::vector<double> take_update_nis() { return {}; }

 protected:
  IEstimator() = default;
};

class IController {
 public:
  virtual ~IController() = default;
  IController(const IController&) = delete;
  IController& operator=(const IController&) = delete;
  IController(IController&&) = delete;
  IController& operator=(IController&&) = delete;

  // Runs on the estimated state only. `latest_uss` holds the newest message per sensor.
  [[nodiscard]] virtual VehicleCmd compute(SimTime now, const EstimatedState& estimate,
                                           const Path& path,
                                           gsl::span<const UssMsg> latest_uss) = 0;

 protected:
  IController() = default;
};

class ISnapshotSink {
 public:
  virtual ~ISnapshotSink() = default;
  ISnapshotSink(const ISnapshotSink&) = delete;
  ISnapshotSink& operator=(const ISnapshotSink&) = delete;
  ISnapshotSink(ISnapshotSink&&) = delete;
  ISnapshotSink& operator=(ISnapshotSink&&) = delete;

  virtual void on_snapshot(const Snapshot& snapshot) = 0;

 protected:
  ISnapshotSink() = default;
};

enum class PlanStatus : std::uint8_t {
  Ok,
  InvalidInput,  // a slot or vehicle that cannot be used, a start pose that is not finite
  NoSolution,    // the geometry of the start pose and the slot admits no maneuver
  Collision      // every candidate maneuver touches the prior map
};

[[nodiscard]] inline const char* to_string(PlanStatus status) {
  switch (status) {
    case PlanStatus::Ok:
      return "Ok";
    case PlanStatus::InvalidInput:
      return "InvalidInput";
    case PlanStatus::NoSolution:
      return "NoSolution";
    case PlanStatus::Collision:
      return "Collision";
  }
  return "?";
}

struct PlanResult {
  PlanStatus status{PlanStatus::NoSolution};
  Path path;  // empty unless status == Ok; speed profile included
};

class IPathPlanner {
 public:
  virtual ~IPathPlanner() = default;
  IPathPlanner(const IPathPlanner&) = delete;
  IPathPlanner& operator=(const IPathPlanner&) = delete;
  IPathPlanner(IPathPlanner&&) = delete;
  IPathPlanner& operator=(IPathPlanner&&) = delete;

  // The maneuver from `start` (a standstill) into `slot` that stays clear of the prior map.
  [[nodiscard]] virtual PlanResult plan(const Pose2D& start, const ParkingSlot& slot,
                                        const PriorMap& map) = 0;

 protected:
  IPathPlanner() = default;
};

}  // namespace psim
