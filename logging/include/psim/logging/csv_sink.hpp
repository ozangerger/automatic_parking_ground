#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "psim/msgs/interfaces.hpp"

namespace psim {

// Writes one CSV row per snapshot. Units are SI (m, s, rad, m/s); numbers use the classic
// locale. Columns:
//   t_s
//   truth_x truth_y truth_yaw truth_v truth_steer
//   est_x est_y est_yaw est_v          estimated state
//   p_x p_y p_yaw p_v                  diagonal of the estimate covariance
//   pos_err yaw_err v_err              estimate minus truth (position error is the distance)
//   cmd_steer cmd_accel cmd_gear       command of this tick (gear as a name)
//   min_uss                            shortest ultrasonic range of this tick, empty without echo
class CsvSink final : public ISnapshotSink {
 public:
  // Creates missing parent directories. Throws std::runtime_error when the file cannot be opened.
  explicit CsvSink(const std::filesystem::path& file);

  void on_snapshot(const Snapshot& snapshot) override;

  void flush();

  [[nodiscard]] std::size_t rows() const { return rows_; }

  // Header names in file order.
  [[nodiscard]] static const std::vector<std::string>& columns();

 private:
  std::ofstream out_;
  std::size_t rows_{0};
};

}  // namespace psim
