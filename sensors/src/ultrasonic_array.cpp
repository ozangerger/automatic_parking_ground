#include "psim/sensors/ultrasonic_array.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <gsl/narrow>
#include <optional>
#include <stdexcept>

#include "psim/geometry/raycast.hpp"

namespace psim {

namespace {

constexpr double kHalf = 0.5;

}  // namespace

std::vector<UssMount> default_uss_layout() {
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): the mounting table of PLAN.md 7.4.
  const std::array<double, 4> side_y{-0.8, -0.3, 0.3, 0.8};
  std::vector<UssMount> mounts;
  for (const double y : side_y) {
    mounts.push_back({3.55, y, 0.0});
  }
  for (const double y : side_y) {
    mounts.push_back({-0.85, y, kPi});
  }
  mounts.push_back({3.0, 0.9, kHalf * kPi});
  mounts.push_back({-0.5, 0.9, kHalf * kPi});
  mounts.push_back({3.0, -0.9, -kHalf * kPi});
  mounts.push_back({-0.5, -0.9, -kHalf * kPi});
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  return mounts;
}

UltrasonicArray::UltrasonicArray(const Map& map, const UltrasonicParams& params,
                                 std::uint64_t master_seed)
    : SensorBase{params.latency}, map_{&map}, params_{params} {
  const bool ok = !params.mounts.empty() && params.min_range > 0.0 &&
                  params.max_range > params.min_range && params.sigma_abs >= 0.0 &&
                  params.sigma_rel >= 0.0 && params.rays >= 1 && params.beam_half_angle >= 0.0 &&
                  params.dropout >= 0.0 && params.dropout <= 1.0 && params.outlier >= 0.0 &&
                  params.outlier <= 1.0;
  if (!ok) {
    throw std::invalid_argument("UltrasonicArray: invalid parameters");
  }
  for (std::size_t i = 0; i < params.mounts.size(); ++i) {
    rngs_.emplace_back(master_seed, kStreamUssFirst + i);
  }
}

Pose2D UltrasonicArray::mount_pose(const VehicleTruth& truth, std::size_t id) const {
  const UssMount& mount = params_.mounts.at(id);
  const double c = std::cos(truth.pose.yaw);
  const double s = std::sin(truth.pose.yaw);
  return Pose2D{truth.pose.x + c * mount.x - s * mount.y, truth.pose.y + s * mount.x + c * mount.y,
                wrap_angle(truth.pose.yaw + mount.heading)};
}

TimedSensorMsg UltrasonicArray::sample(const VehicleTruth& truth, std::size_t id) {
  const Pose2D origin = mount_pose(truth, id);  // throws for a bad id
  Rng& rng = rngs_[id];

  // Shortest hit over the rays across the beam.
  std::optional<double> nearest;
  for (int i = 0; i < params_.rays; ++i) {
    const double fraction =
        params_.rays == 1 ? 0.0 : 2.0 * i / static_cast<double>(params_.rays - 1) - 1.0;
    const Ray ray =
        Ray::from_angle({origin.x, origin.y}, origin.yaw + fraction * params_.beam_half_angle);
    const std::optional<RayHit> hit = map_->raycast(ray, params_.max_range);
    if (hit.has_value() && (!nearest.has_value() || hit->distance < nearest.value_or(0.0))) {
      nearest = hit->distance;
    }
  }

  // Fixed number of draws per measurement, whatever the outcome.
  const double dropout_u = rng.uniform();
  const double outlier_u = rng.uniform();
  const double outlier_value_u = rng.uniform();
  const double noise_z = rng.gaussian();

  UssMsg msg;
  msg.stamp = truth.t;
  msg.id = gsl::narrow<std::uint8_t>(id);
  if (dropout_u < params_.dropout) {
    // no echo
  } else if (outlier_u < params_.outlier) {
    msg.range = params_.min_range + outlier_value_u * (params_.max_range - params_.min_range);
  } else if (nearest.has_value()) {
    const double range = std::max(nearest.value_or(0.0), params_.min_range);
    const double noisy = range + noise_z * (params_.sigma_abs + params_.sigma_rel * range);
    msg.range = std::clamp(noisy, params_.min_range, params_.max_range);
  }
  return timed(truth.t, msg);
}

std::vector<TimedSensorMsg> UltrasonicArray::sample_all(const VehicleTruth& truth) {
  std::vector<TimedSensorMsg> out;
  out.reserve(size());
  for (std::size_t id = 0; id < size(); ++id) {
    out.push_back(sample(truth, id));
  }
  return out;
}

}  // namespace psim
