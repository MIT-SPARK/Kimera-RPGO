#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <chrono>
#include <string>

#include "kimera_rpgo/solver/solver_config.h"

namespace kimera_rpgo {

struct SolverLog {
  size_t num_variables;
  size_t num_factors;
  std::chrono::milliseconds elapsed;
  void print() const;
};

class Solver {
 public:
  Solver(const SolverConfig& config);
  virtual ~Solver();

  gtsam::Values solve(const gtsam::NonlinearFactorGraph& factors,
                      const gtsam::Values& initial,
                      std::vector<double>& weights);

  void clear();

  inline const SolverLog& getLog() { return log_; }

  inline void setKnownInliers(const std::set<size_t>& indices) {
    known_inliers_ = indices;
  }

  inline void setCorruptedOdom(const std::set<size_t>& indices) {
    corrupted_odom_indices_ = indices;
  }

 protected:
  virtual gtsam::Values optimize(const gtsam::NonlinearFactorGraph& factors,
                                 const gtsam::Values& initial,
                                 std::vector<double>& weights) = 0;

 protected:
  SolverLog log_;
  SolverConfig config_;
  std::set<size_t> known_inliers_;
  std::set<size_t> corrupted_odom_indices_;
};

}  // namespace kimera_rpgo
