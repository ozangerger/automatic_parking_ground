#pragma once

namespace psim {

// Mounting position and boresight direction of an ultrasonic sensor in the vehicle frame
// (x forward from the rear axle, y left). Shared by the sensor model and by the ECU, which
// knows where its sensors sit.
struct UssMount {
  double x{0.0};
  double y{0.0};
  double heading{0.0};  // rad, relative to the vehicle x axis
};

}  // namespace psim
