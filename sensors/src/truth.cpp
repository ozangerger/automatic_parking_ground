#include "psim/sensors/truth.hpp"

namespace psim {

VehicleTruth make_truth(SimTime t, const KinematicBicycleModel& model) {
  VehicleTruth truth;
  truth.t = t;
  truth.pose = model.state().pose;
  truth.v = model.state().v;
  truth.steer_angle = model.steer_angle();
  truth.accel = model.accel();
  truth.yaw_rate = model.yaw_rate();
  truth.lateral_accel = model.lateral_accel();
  return truth;
}

}  // namespace psim
