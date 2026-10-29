#pragma once

#include <atomic>
#include <filesystem>
#include <random>
#include <string>
#include <system_error>

namespace psim {

// A fresh directory under the system temp directory, removed on destruction.
class TempDir {
 public:
  TempDir() {
    static std::atomic<int> counter{0};
    std::random_device device;
    path_ = std::filesystem::temp_directory_path() /
            ("psim_test_" + std::to_string(device()) + "_" + std::to_string(counter++));
    std::filesystem::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  TempDir(TempDir&&) = delete;
  TempDir& operator=(TempDir&&) = delete;

  [[nodiscard]] const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

}  // namespace psim
