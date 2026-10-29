#include "psim/fusion/uss_update.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "psim/geometry/raycast.hpp"

namespace psim {

namespace {

double square(double value) { return value * value; }

}  // namespace

UssUpdate::UssUpdate(std::vector<Polygon> map, std::vector<UssMount> mounts,
                     const UssUpdateParams& params)
    : map_{std::move(map)}, mounts_{std::move(mounts)}, params_{params} {
  const bool ok = !mounts_.empty() && params.min_range > 0.0 &&
                  params.max_range > params.min_range && params.sigma_abs >= 0.0 &&
                  params.sigma_rel >= 0.0 && params.rays >= 1 && params.beam_half_angle >= 0.0 &&
                  params.map_sigma >= 0.0 && params.min_incidence_cos >= 0.0 &&
                  params.min_incidence_cos < 1.0 && params.probe_position > 0.0 &&
                  params.probe_yaw > 0.0 && params.max_prediction_jump > 0.0;
  if (!ok) {
    throw std::invalid_argument("UssUpdate: invalid parameters or no sensors");
  }
}

std::optional<UssUpdate::RawRange> UssUpdate::raw_range(const UssMount& mount, double x, double y,
                                                        double yaw) const {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const Eigen::Vector2d origin{x + c * mount.x - s * mount.y, y + s * mount.x + c * mount.y};
  const double heading = yaw + mount.heading;

  std::optional<RawRange> nearest;
  for (int i = 0; i < params_.rays; ++i) {
    const double fraction =
        params_.rays == 1 ? 0.0 : 2.0 * i / static_cast<double>(params_.rays - 1) - 1.0;
    const Ray ray = Ray::from_angle(origin, heading + fraction * params_.beam_half_angle);
    const std::optional<RayHit> hit = raycast(ray, map_, params_.max_range);
    if (hit.has_value() && (!nearest.has_value() || hit->distance < nearest->range)) {
      nearest = RawRange{hit->distance, hit->incidence_cos(ray)};
    }
  }
  return nearest;
}

std::optional<UssPrediction> UssUpdate::predict(std::size_t sensor, double x, double y,
                                                double yaw) const {
  const UssMount& mount = mounts_.at(sensor);
  const std::optional<RawRange> nominal = raw_range(mount, x, y, yaw);
  if (!nominal.has_value() || nominal->range <= params_.min_range ||
      nominal->incidence_cos < params_.min_incidence_cos) {
    return std::nullopt;
  }

  // Central differences; every probe must still hit, and none may jump (corner or edge).
  UssPrediction prediction;
  prediction.range = nominal->range;
  const std::array<double, 3> steps{params_.probe_position, params_.probe_position,
                                    params_.probe_yaw};
  for (int axis = 0; axis < 3; ++axis) {
    const double h = steps.at(static_cast<std::size_t>(axis));
    const auto probe = [&](double sign) {
      return raw_range(mount, x + (axis == 0 ? sign * h : 0.0), y + (axis == 1 ? sign * h : 0.0),
                       yaw + (axis == 2 ? sign * h : 0.0));
    };
    const std::optional<RawRange> plus = probe(1.0);
    const std::optional<RawRange> minus = probe(-1.0);
    if (!plus.has_value() || !minus.has_value()) {
      return std::nullopt;
    }
    if (std::abs(plus->range - nominal->range) > params_.max_prediction_jump ||
        std::abs(minus->range - nominal->range) > params_.max_prediction_jump) {
      return std::nullopt;
    }
    prediction.gradient(axis) = (plus->range - minus->range) / (h + h);
  }
  return prediction;
}

double UssUpdate::variance(double range) const {
  return square(params_.sigma_abs + params_.sigma_rel * range) + square(params_.map_sigma);
}

bool UssUpdate::usable(double measured) const {
  return measured > params_.min_range && measured <= params_.max_range;
}

}  // namespace psim
