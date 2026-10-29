#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include "psim/fusion/ekf_model.hpp"
#include "psim/fusion/time_align_buffer.hpp"
#include "psim/fusion/uss_update.hpp"
#include "psim/geometry/polygon.hpp"
#include "psim/msgs/interfaces.hpp"
#include "psim/msgs/messages.hpp"
#include "psim/msgs/mounts.hpp"
#include "psim/msgs/types.hpp"

namespace psim {

// What the ECU knows about the vehicle and its sensors (data sheet values), and how much it
// trusts them. Defaults match the PLAN.md sensor and vehicle defaults.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers): this table is where the numbers get names.
struct EkfParams {
  // Start of the estimate.
  Pose2D initial_pose;
  double initial_speed{0.0};  // m/s

  // Vehicle and wheel data.
  double wheelbase{2.7};      // m
  double wheel_radius{0.30};  // m, nominal
  int ticks_per_rev{96};

  // Uncertainty of the start (1 sigma).
  double initial_position_sigma{0.02};  // m
  double initial_yaw_sigma{0.01};       // rad
  // The vehicle starts parked, so the speed is known to be about 0. A loose prior would let the
  // first wheel readings overwrite the accelerometer-integrated speed and hide the wheel scale.
  double initial_speed_sigma{0.005};  // m/s
  double initial_bias_sigma{0.005};   // rad/s
  double initial_scale_sigma{0.01};   // wheel scale error: the wheels differ from the nominal ones

  // Process noise: the IMU. A sample with error sigma every imu_period integrates to a variance of
  // sigma^2 * imu_period * T over a time T.
  double gyro_noise{0.005};        // rad/s per sample
  double accel_noise{0.05};        // m/s^2 per sample
  double gyro_bias_walk{2e-4};     // rad/s per sqrt(s)
  double scale_walk{1e-4};         // wheel scale: 1 per sqrt(s), it hardly changes within a run
  SimTime imu_period{10'000'000};  // 100 Hz

  // Measurement noise.
  double steer_noise{0.002};  // rad
  // rad/s: how well v tan(delta) / L describes the yaw rate. The simulated plant is exactly this
  // model, so this is only a safety margin; a real vehicle (tire slip) needs a larger value.
  double steer_model_sigma{0.003};
  double odom_slip_sigma{0.002};  // m per sqrt(m)

  // Wheel ticks are turned into one speed measurement per window.
  SimTime odom_window{200'000'000};  // 0.2 s

  // Updates with a normalized innovation squared above this are rejected
  // (chi-square, 1 degree of freedom, 99 %).
  double gate{6.635};

  // Ultrasonic updates against the ECU's own copy of the map. Both must be non-empty to enable
  // them; the polygons are in the world frame, the mounts in the vehicle frame.
  std::vector<Polygon> map;
  std::vector<UssMount> uss_mounts;
  UssUpdateParams uss;

  // Retrodiction (TimeAlignBuffer): how far back the filter can rewind to apply a message that
  // arrives after the state has moved on. Zero: such messages are applied at the current time.
  SimTime time_align_horizon{500'000'000};
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

struct EkfStats {
  std::size_t odometry_accepted{0};
  std::size_t odometry_rejected{0};
  std::size_t steering_accepted{0};
  std::size_t steering_rejected{0};
  std::size_t uss_accepted{0};
  std::size_t uss_rejected{0};
  std::size_t uss_skipped{0};    // echoes without a usable prediction (miss, grazing, corner, ...)
  double odometry_nis_sum{0.0};  // NIS of accepted and rejected updates, per kind
  double steering_nis_sum{0.0};
  double uss_nis_sum{0.0};
  std::optional<double> last_nis;
  std::size_t late_replays{0};      // late messages applied by rewinding and replaying
  std::size_t late_applied_now{0};  // late messages beyond the horizon, applied at the current time

  [[nodiscard]] std::size_t odometry_count() const { return odometry_accepted + odometry_rejected; }
  [[nodiscard]] std::size_t steering_count() const { return steering_accepted + steering_rejected; }
  [[nodiscard]] std::size_t uss_count() const { return uss_accepted + uss_rejected; }
  // The NIS of a consistent filter averages the degrees of freedom of the update (1 here).
  [[nodiscard]] double mean_odometry_nis() const {
    return odometry_count() == 0 ? 0.0 : odometry_nis_sum / static_cast<double>(odometry_count());
  }
  [[nodiscard]] double mean_steering_nis() const {
    return steering_count() == 0 ? 0.0 : steering_nis_sum / static_cast<double>(steering_count());
  }
  [[nodiscard]] double mean_uss_nis() const {
    return uss_count() == 0 ? 0.0 : uss_nis_sum / static_cast<double>(uss_count());
  }
};

// Extended Kalman filter on [x, y, yaw, v, gyro bias, odometry scale] (rear axle reference).
//
// Predict: the IMU (gyro yaw rate, accelerometer x) drives the motion model; a new IMU message
// advances the state to its stamp, predict_to() extrapolates with the latest IMU values.
// Updates (each a scalar update with a chi-square gate):
//  - wheel odometry: mean speed over a window of ticks (compensated for acceleration) equals
//    scale * v. The wheels differ from the nominal ones by a constant error; without the scale
//    state that error is mistaken for gyro bias (the steering update below compares the gyro
//    with v tan(delta) / L). The scale is observable from the accelerometer while the vehicle
//    accelerates;
//  - steering angle: the gyro reading must equal bias + v tan(delta) / L, which makes the
//    gyro bias observable while driving, driving straight and standing still;
//  - ultrasonic echoes (optional): the measured range must equal the range predicted by casting
//    the sensor's rays from the estimated pose against the prior map (UssUpdate).
//
// A message older than the state (sensor latency) is applied at its own time: the filter rewinds
// to a checkpoint before it and replays everything since, in time order (TimeAlignBuffer).
// Beyond the buffer's horizon, it is applied at the current time. Messages older than the
// newest of their kind are ignored.
class Ekf final : public IEstimator {
 public:
  // Throws std::invalid_argument for non-positive vehicle data, periods or gate, and for an
  // invalid ultrasonic configuration.
  explicit Ekf(const EkfParams& params);

  void on_message(const SensorMsg& msg) override;
  void predict_to(SimTime t) override;

  // The first four states and their covariance block at the state time.
  [[nodiscard]] EstimatedState estimate() const override;

  // NIS of the updates since the last call. Each update is reported once, even when a late
  // message makes the filter replay it.
  [[nodiscard]] std::vector<double> take_update_nis() override;

  [[nodiscard]] double gyro_bias() const { return core_.x(kEkfBias); }
  [[nodiscard]] double odometry_scale() const { return core_.x(kEkfScale); }
  // Covariance of all six states (estimate() only reports the first four).
  [[nodiscard]] const EkfMatrix& full_covariance() const { return core_.p; }
  [[nodiscard]] bool uses_ultrasonics() const { return uss_ != nullptr; }
  [[nodiscard]] EkfStats stats() const;

 private:
  enum class UpdateKind : std::uint8_t { Odometry, Steering, Uss };

  struct TickMark {
    SimTime stamp;
    std::int64_t ticks_rl;
    std::int64_t ticks_rr;
  };

  // Identifies one update: stamp of the message in ns, kind, sensor id.
  using UpdateKey = std::tuple<std::int64_t, std::uint8_t, std::uint8_t>;

  // Everything needed to resume the filter at a point in time; copied into checkpoints.
  struct Core {
    EkfVector x{EkfVector::Zero()};
    EkfMatrix p{EkfMatrix::Zero()};
    SimTime time{};
    bool time_known{false};  // the state time is set by the first message

    EkfInput input;
    std::optional<SimTime> last_imu_stamp;
    std::optional<SimTime> last_steer_stamp;
    std::optional<TickMark> window_start;
    std::vector<std::optional<SimTime>> uss_last_stamp;

    EkfStats stats;
  };

  void on_late_message(const SensorMsg& msg, SimTime stamp);
  void process(const SensorMsg& msg);
  void dispatch(const SensorMsg& msg);
  void on_imu(const ImuMsg& msg);
  void on_odometry(const WheelOdomMsg& msg);
  void on_steering(const SteerMsg& msg);
  void on_uss(const UssMsg& msg);

  // Advances to `to` (if later) holding `input`, in steps of at most 50 ms.
  void propagate(SimTime to, const EkfInput& input);
  void advance_to(SimTime stamp) { propagate(stamp, core_.input); }

  // Scalar measurement update; returns whether the update passed the gate.
  bool scalar_update(UpdateKind kind, std::uint8_t id, SimTime stamp, double innovation,
                     const EkfVector& h, double variance);

  EkfParams params_;
  double meters_per_tick_;
  std::unique_ptr<const UssUpdate> uss_;

  Core core_;
  TimeAlignBuffer<Core> history_;
  std::size_t late_replays_{0};
  std::size_t late_applied_now_{0};

  // Diagnostics live outside the core on purpose: checkpoints must not resurrect them.
  // NIS of the updates not yet taken (a replayed update overwrites its earlier value) and the
  // keys of those already handed out, so that a replay never reports an update twice.
  std::map<UpdateKey, double> pending_nis_;
  std::set<UpdateKey> reported_;
};

}  // namespace psim
