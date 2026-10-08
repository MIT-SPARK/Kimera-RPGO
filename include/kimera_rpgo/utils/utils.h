#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <string>
namespace kimera_rpgo {

template <typename T, typename Ptr>
T* factor_pointer_cast(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get());
}

template <typename T, typename Ptr>
bool factor_is_underlying_type(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get()) != nullptr;
}

void saveG2o(const gtsam::NonlinearFactorGraph& graph,
             const gtsam::Values& estimate,
             const std::string& filename);
}  // namespace kimera_rpgo
