// Authors: Yun Chang
#pragma once

#include <functional>
#include <memory>
#include <utility>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "KimeraRPGO/Logger.h"

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

struct Edge {
  gtsam::Symbol from_key;
  gtsam::Symbol to_key;

  Edge(gtsam::Symbol from, gtsam::Symbol to) : from_key(from), to_key(to) {}
};
typedef std::unique_ptr<const Edge> EdgePtr;

// struct storing the involved parties (ex robot a and robot b)
struct ObservationId {
  char id1;
  char id2;

  ObservationId(char first, char second) {
    id1 = first;
    id2 = second;
  }

  bool operator==(const ObservationId& other) const {
    if (id1 == other.id1 && id2 == other.id2) return true;
    if (id2 == other.id1 && id1 == other.id2) return true;
    return false;
  }
};

// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename T, typename Ptr>
T* factor_pointer_cast(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get());
}

// TODO(nathan) there is probably a better way to do this
template <typename T, typename Ptr>
bool factor_is_underlying_type(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get()) != nullptr;
}

}  // namespace KimeraRPGO

namespace std {
// hash function for ObservationId
template <>
struct hash<KimeraRPGO::ObservationId> {
  std::size_t operator()(const KimeraRPGO::ObservationId& id) const {
    using std::hash;
    return hash<char>()(id.id1) + hash<char>()(id.id2) +
           hash<char>()(id.id1) * hash<char>()(id.id2);
  }
};
}  // namespace std
