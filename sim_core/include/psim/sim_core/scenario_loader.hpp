#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string_view>

#include "psim/sim_core/scenario.hpp"

namespace psim {

// Thrown for unreadable files, malformed JSON, and missing or invalid fields.
class ScenarioError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Parses a scenario from JSON text. Units are SI; angles are radians; `duration_s` is seconds.
[[nodiscard]] Scenario parse_scenario(std::string_view json_text);

// "pure_pursuit", "stanley" or "kanayama" (the names used in scenario files and on command lines).
[[nodiscard]] std::optional<ControllerKind> controller_from_name(std::string_view name);

// Reads and parses a scenario file.
[[nodiscard]] Scenario load_scenario(const std::filesystem::path& file);

}  // namespace psim
