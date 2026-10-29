#include "psim/sensors/gnss_ref.hpp"

#include <stdexcept>

namespace psim {

GnssRef::GnssRef(const GnssParams& params, std::uint64_t master_seed)
    : SensorBase{SimTime::zero()}, params_{params}, rng_{master_seed, kStreamGnss} {
  if (params.sigma < 0.0) {
    throw std::invalid_argument("GnssRef: sigma must not be negative");
  }
}

GnssMsg GnssRef::sample(const VehicleTruth& truth) {
  GnssMsg msg;
  msg.stamp = truth.t;
  msg.x = rng_.gaussian(truth.pose.x, params_.sigma);
  msg.y = rng_.gaussian(truth.pose.y, params_.sigma);
  return msg;
}

}  // namespace psim
