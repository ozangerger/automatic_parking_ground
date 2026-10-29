#include "psim/logging/scene.hpp"

#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace psim {

namespace {

constexpr int kPrecision = 10;
constexpr unsigned char kFirstPrintable = 0x20;  // JSON escapes everything below

void write_string(std::ostream& out, const std::string& text) {
  out << '"';
  for (const char c : text) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < kFirstPrintable) {  // other control characters
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c)) << std::dec << std::setfill(' ');
        } else {
          out << c;
        }
    }
  }
  out << '"';
}

void write_points(std::ostream& out, const std::vector<ScenePoint>& points) {
  out << '[';
  for (std::size_t i = 0; i < points.size(); ++i) {
    out << (i == 0 ? "" : ", ") << '[' << points[i].x << ", " << points[i].y << ']';
  }
  out << ']';
}

}  // namespace

std::string scene_to_json(const StaticScene& scene) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(kPrecision);

  out << "{\n  \"name\": ";
  write_string(out, scene.name);

  out << ",\n  \"obstacles\": [";
  for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
    out << (i == 0 ? "\n    " : ",\n    ") << "{\"name\": ";
    write_string(out, scene.obstacles[i].name);
    out << ", \"vertices\": ";
    write_points(out, scene.obstacles[i].vertices);
    out << '}';
  }
  out << (scene.obstacles.empty() ? "]" : "\n  ]");

  if (scene.slot.has_value()) {
    out << ",\n  \"slot\": {\"vertices\": ";
    write_points(out, scene.slot->vertices);
    if (scene.slot_pose.has_value()) {
      out << ", \"pose\": {\"x\": " << scene.slot_pose->x << ", \"y\": " << scene.slot_pose->y
          << ", \"yaw\": " << scene.slot_pose->yaw << '}';
    }
    out << '}';
  }

  if (!scene.car_footprint.empty()) {
    out << ",\n  \"car\": ";
    write_points(out, scene.car_footprint);
  }
  if (!scene.path.empty()) {
    out << ",\n  \"path\": ";
    write_points(out, scene.path);
  }
  out << "\n}\n";
  return out.str();
}

void write_scene(const StaticScene& scene, const std::filesystem::path& file) {
  if (file.has_parent_path()) {
    std::filesystem::create_directories(file.parent_path());
  }
  std::ofstream out{file};
  if (!out) {
    throw std::runtime_error("write_scene: cannot open " + file.string());
  }
  out << scene_to_json(scene);
  if (!out) {
    throw std::runtime_error("write_scene: write failed for " + file.string());
  }
}

}  // namespace psim
