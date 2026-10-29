#pragma once

#include <gsl/span>

#include <vector>

#include "psim/msgs/estimate.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/path.hpp"
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

// IPathPlanner is added in step 2.4: its signature needs a prior-map type that the
// dependency rules do not yet provide (see PLAN.md section 6).

}  // namespace psim
