#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "psim/logging/scene.hpp"
#include "psim/msgs/interfaces.hpp"

namespace psim {

struct RerunSinkOptions {
  std::string application_id{"parking_sim"};
  // Where to save the recording for replay with the viewer (rerun FILE.rrd).
  std::optional<std::filesystem::path> rrd_file;
  // Start a viewer (the `rerun` executable of the same version as the SDK, on the PATH) and stream
  // to it. At least one of rrd_file and spawn_viewer is needed.
  bool spawn_viewer{false};
  // Rerun's 2D views have y pointing down (like an image), so the world would appear mirrored:
  // y is negated for display. Hover values in the viewer then show the negated y. The plots are
  // not affected.
  bool flip_y{true};
  double uss_no_echo_length{4.0};  // m: how long a beam without an echo is drawn
  // Beams without an echo (nothing within range, or a dropout) are drawn dim and thin; false hides
  // them. They are not detections.
  bool draw_no_echo_beams{true};
  int trail_every{5};  // snapshots between updates of the two trails
  int ellipse_points{48};
  double ellipse_sigmas{2.0};
};

// Shows a run in the Rerun viewer: the world and the slot (static), the truth and the estimated car
// with their trails, the 2-sigma position ellipse, the ultrasonic beams, the planned path, and
// scalar plots (position error, yaw error, NIS, steering command and actual, speed). Time is the
// simulation time, as the "sim_time" timeline. The Rerun headers stay out of this header.
//
// Entities: world/obstacles/*, world/slot, world/truth_car, world/est_car, world/trail_truth,
// world/trail_est, world/est_ellipse, world/uss_echoes, world/uss_no_echo, world/path,
// plots/pos_err (m), plots/yaw_err (deg), plots/nis, plots/steer_cmd (deg), plots/steer_actual
// (deg), plots/speed (m/s).
class RerunSink final : public ISnapshotSink {
 public:
  // Logs the static scene. Throws std::invalid_argument without an output (see RerunSinkOptions)
  // and std::runtime_error when the file cannot be created or the viewer cannot be started.
  RerunSink(const StaticScene& scene, const RerunSinkOptions& options);
  ~RerunSink() override;

  void on_snapshot(const Snapshot& snapshot) override;

  [[nodiscard]] std::size_t snapshots() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace psim
