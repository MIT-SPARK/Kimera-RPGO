// Authors: Yun Chang

#ifndef KIMERARPGO_UTILS_TYPE_UTILS_H_
#define KIMERARPGO_UTILS_TYPE_UTILS_H_

#include <memory>
#include <utility>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "KimeraRPGO/logger.h"

namespace KimeraRPGO {

struct Measurements {
  gtsam::NonlinearFactorGraph factors;
  gtsam::NonlinearFactorGraph consistent_factors;
  gtsam::Matrix adj_matrix;
  gtsam::Matrix dist_matrix;

  Measurements(
      gtsam::NonlinearFactorGraph new_factors = gtsam::NonlinearFactorGraph())
      : factors(new_factors), consistent_factors(new_factors) {
    if (new_factors.size() > 1) {
      log<WARNING>(
          "Unexpected behavior: initializing Measurement struct with more than "
          "one factor.");
    }
    adj_matrix = Eigen::MatrixXd::Zero(1, 1);
    dist_matrix = Eigen::MatrixXd::Zero(1, 1);
  }
};

// struct storing the involved parties (ex robot a and robot b)
struct ObservationId {
  char id1;
  char id2;

  ObservationId(char first, char second) {
    id1 = first;
    id2 = second;
  }

  bool operator==(const ObservationId& a, const ObservationId& b) {
    if (a.id1 == b.id1 && a.id2 == b.id2) return true;
    if (a.id2 == b.id1 && a.id1 == b.id2) return true;
    return false;
  }
};

// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace KimeraRPGO

#endif  // KIMERARPGO_UTILS_TYPE_UTILS_H_
