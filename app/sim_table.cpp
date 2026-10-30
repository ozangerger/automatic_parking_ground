// Runs a scenario over several seeds, estimators and controller state sources and reports the
// metrics: a Markdown table on stdout and, in the output directory, table.md and runs.csv.
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "psim/app/metrics_table.hpp"
#include "psim/sim_core/scenario_loader.hpp"

namespace {

void print_usage() {
  std::cerr
      << "usage: sim_table SCENARIO.json [--seeds N] [--first-seed S]\n"
         "                 [--estimators ekf,dead_reckoning] [--states estimate,truth]\n"
         "                 [--controllers pure_pursuit,stanley,kanayama] [--out DIR]\n"
         "  Runs every combination for the seeds first-seed .. first-seed + N - 1 and prints\n"
         "  the metrics table. Writes DIR/table.md and DIR/runs.csv\n"
         "  (default DIR: out/<scenario name>). Defaults: 5 seeds from 1, both estimators,\n"
         "  the controller state and the controller of the scenario.\n";
}

std::vector<std::string> split(const std::string& text) {
  std::vector<std::string> parts;
  std::istringstream stream{text};
  std::string part;
  while (std::getline(stream, part, ',')) {
    parts.push_back(part);
  }
  return parts;
}

void write_file(const std::filesystem::path& file, const std::string& text) {
  std::filesystem::create_directories(file.parent_path());
  std::ofstream out{file};
  out << text;
  if (!out) {
    throw std::runtime_error("cannot write " + file.string());
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path scenario_file;
  std::filesystem::path out_dir;
  psim::BatchOptions options;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      const bool has_value = i + 1 < argc;
      if (arg == "--seeds" && has_value) {
        options.seeds =
            std::stoi(argv[++i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      } else if (arg == "--first-seed" && has_value) {
        options.first_seed =
            std::stoull(argv[++i]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      } else if (arg == "--out" && has_value) {
        out_dir = argv[++i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      } else if (arg == "--estimators" && has_value) {
        options.estimators.clear();
        for (const std::string& name :
             split(argv[++i])) {  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          if (name == "ekf") {
            options.estimators.push_back(psim::EstimatorKind::Ekf);
          } else if (name == "dead_reckoning") {
            options.estimators.push_back(psim::EstimatorKind::DeadReckoning);
          } else {
            print_usage();
            return 2;
          }
        }
      } else if (arg == "--controllers" && has_value) {
        options.controllers.clear();
        for (const std::string& name :
             split(argv[++i])) {  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          const std::optional<psim::ControllerKind> kind = psim::controller_from_name(name);
          if (!kind.has_value()) {
            print_usage();
            return 2;
          }
          options.controllers.push_back(kind.value_or(psim::ControllerKind::PurePursuit));
        }
      } else if (arg == "--states" && has_value) {
        options.controller_states.clear();
        for (const std::string& name :
             split(argv[++i])) {  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          if (name == "estimate") {
            options.controller_states.push_back(psim::StateSource::Estimate);
          } else if (name == "truth") {
            options.controller_states.push_back(psim::StateSource::Truth);
          } else {
            print_usage();
            return 2;
          }
        }
      } else if (scenario_file.empty() && arg.rfind("--", 0) != 0) {
        scenario_file = arg;
      } else {
        print_usage();
        return 2;
      }
    }
    if (scenario_file.empty()) {
      print_usage();
      return 2;
    }

    const psim::Scenario scenario = psim::load_scenario(scenario_file);
    if (out_dir.empty()) {
      out_dir = std::filesystem::path{"out"} / scenario.name;
    }
    const std::vector<psim::RunRecord> records = psim::run_batch(scenario, options);
    const std::string table = psim::format_table(records, scenario.acceptance);
    write_file(out_dir / "table.md", "# " + scenario.name + "\n\n" + table);
    write_file(out_dir / "runs.csv", psim::format_csv(records));
    write_file(out_dir / "comfort.md",
               "# " + scenario.name + ": ride comfort\n\n" + psim::format_comfort_table(records));
    std::cout << "# " << scenario.name << " (" << records.size() << " runs)\n\n"
              << table << "\nwrote " << (out_dir / "table.md").string()
              << ", comfort.md and runs.csv\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "sim_table: " << e.what() << '\n';
    return 1;
  }
}
