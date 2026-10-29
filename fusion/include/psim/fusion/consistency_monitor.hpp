#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <deque>
#include <vector>

namespace psim {

// Is the filter's idea of its own accuracy right? The ECU side can check the innovations of its
// updates (NIS) and the sanity of its covariance. The comparison with the truth (NEES) needs the
// truth and therefore lives in the evaluation code (logging/nees.hpp), never here.
//
// A consistent filter has a NIS that follows a chi-square distribution with 1 degree of freedom
// for a scalar update: mean 1, and 95 % of the values below 3.84.
class ConsistencyMonitor {
 public:
  // Upper bound containing 95 % of a chi-square variable with 1 degree of freedom.
  static constexpr double kChiSquare95OneDof = 3.841458820694124;
  static constexpr double kChiSquareMedianOneDof = 0.45493642311957283;
  static constexpr std::size_t kDefaultWindow = 100;
  static constexpr double kDefaultSigmas = 4.0;
  static constexpr double kSymmetryTolerance = 1e-9;  // relative to the largest entry

  // `window` is the number of most recent values behind recent_mean_nis(); must be positive.
  explicit ConsistencyMonitor(std::size_t window = kDefaultWindow);

  void add_nis(double nis);

  [[nodiscard]] std::size_t count() const { return count_; }
  [[nodiscard]] double mean_nis() const;         // all values; 0 without any
  [[nodiscard]] double recent_mean_nis() const;  // the last `window` values; 0 without any
  // The median; unlike the mean it is not dragged away by a few rejected outliers. A chi-square
  // variable with 1 degree of freedom has median 0.455. 0 without any value.
  [[nodiscard]] double median_nis() const;
  // Share of the NIS values at or below the 95 % bound; 1 without any value.
  [[nodiscard]] double fraction_inside_bound() const;

  // True when the mean NIS lies within `sigmas` standard errors of 1, assuming independent
  // updates (a rough test: neighbouring updates are correlated, so keep `sigmas` generous).
  [[nodiscard]] bool mean_is_plausible(double sigmas = kDefaultSigmas) const;

  // A covariance is sane when it is symmetric (within `tolerance`, relative), positive definite
  // and every standard deviation stays below `max_sigma`.
  [[nodiscard]] static bool covariance_is_sane(const Eigen::Ref<const Eigen::MatrixXd>& covariance,
                                               double max_sigma,
                                               double tolerance = kSymmetryTolerance);

 private:
  std::size_t window_;
  std::size_t count_{0};
  std::size_t inside_{0};
  double sum_{0.0};
  std::vector<double> all_;
  std::deque<double> recent_;
  double recent_sum_{0.0};
};

}  // namespace psim
