#include "psim/logging_rerun/rerun_sink.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <rerun.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "psim/logging/viewer_geometry.hpp"
#include "psim/msgs/math.hpp"

namespace psim {

namespace {

// The colors of tools/plot.py: truth = categorical slot 1, estimate = slot 2, the third series
// (here the steering actual) = slot 3.
const rerun::Color kTruth{0x2a, 0x78, 0xd6};
const rerun::Color kEstimate{0xeb, 0x68, 0x34};
const rerun::Color kThird{0x1b, 0xaf, 0x7a};
const rerun::Color kObstacle{0x9a, 0x99, 0x92};
const rerun::Color kInk{0x52, 0x51, 0x4e};
// Beams without an echo (free space, a dropout) are context, not detections: dim and thin.
const rerun::Color kNoEcho{0x4a, 0x49, 0x46};

constexpr float kLineWidth = 1.5F;
constexpr float kThinLineWidth = 0.8F;
constexpr float kNoEchoWidth = 0.4F;
constexpr float kPlotWidth = 1.2F;
constexpr double kRadToDeg = 180.0 / kPi;

// The one place where world coordinates become view coordinates. Rerun's 2D views have y pointing
// down, like an image; the world has y to the left of x, so y is negated to keep left turns
// looking like left turns (RerunSinkOptions::flip_y).
rerun::LineStrip2D make_strip(const std::vector<ScenePoint>& points, float y_sign, bool closed) {
  std::vector<rerun::Vec2D> vertices;
  vertices.reserve(points.size() + 1U);
  for (const ScenePoint& p : points) {
    vertices.emplace_back(static_cast<float>(p.x), y_sign * static_cast<float>(p.y));
  }
  if (closed && !points.empty()) {
    vertices.push_back(vertices.front());
  }
  return rerun::LineStrip2D{std::move(vertices)};
}

std::string entity_name(const std::string& name, std::size_t index) {
  std::string result;
  for (const char c : name) {
    result += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
  }
  return result.empty() ? "obstacle_" + std::to_string(index) : result;
}

bool same_path(const Path& a, const Path& b) {
  if (a.points.size() != b.points.size()) {
    return false;
  }
  if (a.points.empty()) {
    return true;
  }
  return a.points.front().x == b.points.front().x && a.points.back().x == b.points.back().x &&
         a.points.back().y == b.points.back().y;
}

}  // namespace

struct RerunSink::Impl {
  Impl(const StaticScene& scene_in, const RerunSinkOptions& options_in)
      : rec{options_in.application_id}, scene{scene_in}, options{options_in} {}

  rerun::RecordingStream rec;
  StaticScene scene;
  RerunSinkOptions options;
  std::vector<ScenePoint> truth_trail;
  std::vector<ScenePoint> estimate_trail;
  Path logged_path;
  std::size_t count{0};

  [[nodiscard]] rerun::LineStrip2D strip(const std::vector<ScenePoint>& points,
                                         bool closed = false) const {
    return make_strip(points, options.flip_y ? -1.0F : 1.0F, closed);
  }

  void log_series_style(const std::string& entity, const char* name, const rerun::Color& color) {
    rec.log_static(
        entity, rerun::SeriesLines().with_colors(color).with_names(name).with_widths(kPlotWidth));
  }

  void log_static_scene() {
    for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
      rec.log_static("world/obstacles/" + entity_name(scene.obstacles[i].name, i),
                     rerun::LineStrips2D(strip(scene.obstacles[i].vertices, true))
                         .with_colors(kObstacle)
                         .with_radii(rerun::Radius::ui_points(kLineWidth)));
    }
    if (scene.slot.has_value()) {
      rec.log_static("world/slot", rerun::LineStrips2D(strip(scene.slot->vertices, true))
                                       .with_colors(kInk)
                                       .with_radii(rerun::Radius::ui_points(kLineWidth)));
    }
    log_series_style("plots/pos_err", "position error (m)", kEstimate);
    log_series_style("plots/yaw_err", "yaw error (deg)", kEstimate);
    log_series_style("plots/nis", "NIS", kEstimate);
    log_series_style("plots/steer_cmd", "steering command (deg)", kEstimate);
    log_series_style("plots/steer_actual", "steering actual (deg)", kThird);
    log_series_style("plots/speed", "speed (m/s)", kTruth);
  }

  void log_car(const char* entity, const Pose2D& pose, const rerun::Color& color) {
    rec.log(entity, rerun::LineStrips2D(strip(footprint_at(scene.car_footprint, pose), true))
                        .with_colors(color)
                        .with_radii(rerun::Radius::ui_points(kLineWidth)));
  }

  void log_trail(const char* entity, std::vector<ScenePoint>& trail, const ScenePoint& point,
                 const rerun::Color& color, bool update) {
    trail.push_back(point);
    if (update) {
      rec.log(entity, rerun::LineStrips2D(strip(trail))
                          .with_colors(color)
                          .with_radii(rerun::Radius::ui_points(kThinLineWidth)));
    }
  }

  void on_snapshot(const Snapshot& s) {
    rec.set_time_duration_nanos("sim_time", s.t.count());
    const Pose2D estimate{s.est.x(0), s.est.x(1), s.est.x(2)};
    const bool update_trails =
        options.trail_every <= 1 || count % static_cast<std::size_t>(options.trail_every) == 0;

    if (!scene.car_footprint.empty()) {
      log_car("world/truth_car", s.truth, kTruth);
      log_car("world/est_car", estimate, kEstimate);
    }
    log_trail("world/trail_truth", truth_trail, ScenePoint{s.truth.x, s.truth.y}, kTruth,
              update_trails);
    log_trail("world/trail_est", estimate_trail, ScenePoint{estimate.x, estimate.y}, kEstimate,
              update_trails);

    // An estimator without a covariance (dead reckoning) reports zeros: no ellipse then.
    const double pxx = s.est.P(0, 0);
    const double pxy = s.est.P(0, 1);
    const double pyy = s.est.P(1, 1);
    if (std::isfinite(pxx + pyy + pxy) && pxx + pyy > 0.0) {
      rec.log("world/est_ellipse",
              rerun::LineStrips2D(
                  strip(covariance_ellipse(ScenePoint{estimate.x, estimate.y}, pxx, pxy, pyy,
                                           options.ellipse_sigmas, options.ellipse_points)))
                  .with_colors(kEstimate)
                  .with_radii(rerun::Radius::ui_points(kThinLineWidth)));
    }

    if (!s.uss.empty()) {
      // Two entities, so that the echoes stay readable and the empty beams can be hidden.
      std::vector<rerun::LineStrip2D> echoes;
      std::vector<rerun::LineStrip2D> empty;
      for (const UssRay& ray : s.uss) {
        const SceneSegment segment = ray_segment(ray, options.uss_no_echo_length);
        (ray.range.has_value() ? echoes : empty).push_back(strip({segment.from, segment.to}));
      }
      if (echoes.empty()) {
        rec.log("world/uss_echoes", rerun::Clear::FLAT);
      } else {
        rec.log("world/uss_echoes", rerun::LineStrips2D(std::move(echoes))
                                        .with_colors(kThird)
                                        .with_radii(rerun::Radius::ui_points(kThinLineWidth)));
      }
      if (options.draw_no_echo_beams) {
        if (empty.empty()) {
          rec.log("world/uss_no_echo", rerun::Clear::FLAT);
        } else {
          rec.log("world/uss_no_echo", rerun::LineStrips2D(std::move(empty))
                                           .with_colors(kNoEcho)
                                           .with_radii(rerun::Radius::ui_points(kNoEchoWidth)));
        }
      }
    }

    if (!s.planned_path.points.empty() && !same_path(s.planned_path, logged_path)) {
      std::vector<ScenePoint> points;
      points.reserve(s.planned_path.points.size());
      for (const PathPoint& p : s.planned_path.points) {
        points.push_back(ScenePoint{p.x, p.y});
      }
      rec.log("world/path", rerun::LineStrips2D(strip(points))
                                .with_colors(kInk)
                                .with_radii(rerun::Radius::ui_points(kThinLineWidth)));
      logged_path = s.planned_path;
    }

    rec.log("plots/pos_err",
            rerun::Scalars(std::hypot(estimate.x - s.truth.x, estimate.y - s.truth.y)));
    rec.log("plots/yaw_err", rerun::Scalars(kRadToDeg * wrap_angle(estimate.yaw - s.truth.yaw)));
    if (s.nis.has_value()) {
      rec.log("plots/nis", rerun::Scalars(*s.nis));
    }
    rec.log("plots/steer_cmd", rerun::Scalars(kRadToDeg * s.cmd.steer_angle));
    rec.log("plots/steer_actual", rerun::Scalars(kRadToDeg * s.truth_steer));
    rec.log("plots/speed", rerun::Scalars(s.truth_v));
    ++count;
  }
};

RerunSink::RerunSink(const StaticScene& scene, const RerunSinkOptions& options) {
  if (!options.rrd_file.has_value() && !options.spawn_viewer) {
    throw std::invalid_argument("RerunSink: needs an rrd file or a spawned viewer to send to");
  }
  if (options.trail_every < 1 || options.ellipse_points < 3 || options.ellipse_sigmas < 0.0) {
    throw std::invalid_argument("RerunSink: invalid options");
  }
  impl_ = std::make_unique<Impl>(scene, options);
  if (options.rrd_file.has_value()) {
    const std::filesystem::path parent = options.rrd_file->parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    const rerun::Error error = impl_->rec.save(options.rrd_file->string());
    if (error.is_err()) {
      throw std::runtime_error("RerunSink: cannot save " + options.rrd_file->string() + ": " +
                               error.description);
    }
  }
  if (options.spawn_viewer) {
    const rerun::Error error = impl_->rec.spawn();
    if (error.is_err()) {
      throw std::runtime_error("RerunSink: cannot start the viewer: " + error.description);
    }
  }
  impl_->log_static_scene();
}

RerunSink::~RerunSink() = default;

void RerunSink::on_snapshot(const Snapshot& snapshot) { impl_->on_snapshot(snapshot); }

std::size_t RerunSink::snapshots() const { return impl_->count; }

}  // namespace psim
