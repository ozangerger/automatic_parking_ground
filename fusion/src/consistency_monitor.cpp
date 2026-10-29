#include "psim/fusion/consistency_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace psim {

namespace {
constexpr double kHalf = 0.5;
}  // namespace

ConsistencyMonitor::ConsistencyMonitor(std::size_t window) : window_{window} {
  if (window == 0) {
    throw std::invalid_argument("ConsistencyMonitor: window must be positive");
  }
}

void ConsistencyMonitor::add_nis(double nis) {
  ++count_;
  sum_ += nis;
  all_.push_back(nis);
  if (nis <= kChiSquare95OneDof) {
    ++inside_;
  }
  recent_.push_back(nis);
  recent_sum_ += nis;
  if (recent_.size() > window_) {
    recent_sum_ -= recent_.front();
    recent_.pop_front();
  }
}

double ConsistencyMonitor::mean_nis() const {
  return count_ == 0 ? 0.0 : sum_ / static_cast<double>(count_);
}

double ConsistencyMonitor::recent_mean_nis() const {
  return recent_.empty() ? 0.0 : recent_sum_ / static_cast<double>(recent_.size());
}

double ConsistencyMonitor::median_nis() const {
  if (all_.empty()) {
    return 0.0;
  }
  std::vector<double> sorted = all_;
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
  std::nth_element(sorted.begin(), middle, sorted.end());
  if (sorted.size() % 2 == 1) {
    return *middle;
  }
  const double upper = *middle;
  const double lower = *std::max_element(sorted.begin(), middle);
  return kHalf * (lower + upper);
}

double ConsistencyMonitor::fraction_inside_bound() const {
  return count_ == 0 ? 1.0 : static_cast<double>(inside_) / static_cast<double>(count_);
}

bool ConsistencyMonitor::mean_is_plausible(double sigmas) const {
  if (count_ == 0) {
    return true;
  }
  // A chi-square variable with 1 degree of freedom has variance 2.
  const double standard_error = std::sqrt(2.0 / static_cast<double>(count_));
  return std::abs(mean_nis() - 1.0) <= sigmas * standard_error;
}

bool ConsistencyMonitor::covariance_is_sane(const Eigen::Ref<const Eigen::MatrixXd>& covariance,
                                            double max_sigma, double tolerance) {
  if (covariance.rows() != covariance.cols() || covariance.rows() == 0 || !covariance.allFinite()) {
    return false;
  }
  const double scale = covariance.cwiseAbs().maxCoeff();
  if (scale == 0.0 ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > tolerance * scale) {
    return false;
  }
  const Eigen::LDLT<Eigen::MatrixXd> factorization{covariance};
  if (factorization.info() != Eigen::Success || !factorization.isPositive() ||
      factorization.vectorD().minCoeff() <= 0.0) {
    return false;
  }
  return covariance.diagonal().maxCoeff() <= max_sigma * max_sigma;
}

}  // namespace psim
