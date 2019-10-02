// Authors: Yun Chang

#ifndef KIMERARPGO_UTILS_TYPE_UTILS_H_
#define KIMERARPGO_UTILS_TYPE_UTILS_H_

#include <memory>
#include <utility>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "KimeraRPGO/logger.h"

namespace KimeraRPGO {

struct LandmarkMeasurements {
  gtsam::NonlinearFactorGraph factors;
  gtsam::NonlinearFactorGraph consistent_factors;
  gtsam::Matrix adj_matrix;
  gtsam::Matrix dist_matrix;

  LandmarkMeasurements(
      gtsam::NonlinearFactorGraph new_factors = gtsam::NonlinearFactorGraph())
      : factors(new_factors), consistent_factors(new_factors) {
    if (new_factors.size() > 1) {
      log<WARNING>(
          "Unexpected behavior: initializing landmark with more than "
          "one factor.");
    }
    adj_matrix = Eigen::MatrixXd::Zero(1, 1);
    dist_matrix = Eigen::MatrixXd::Zero(1, 1);
  }
};

// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace KimeraRPGO

#endif  // KIMERARPGO_UTILS_TYPE_UTILS_H_
