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
  using IterationCallback = std::function<void(size_t, double, double)>;

  Solver(const SolverConfig& config);
  virtual ~Solver();

  gtsam::Values solve(const gtsam::NonlinearFactorGraph& factors,
                      const gtsam::Values& initial,
                      std::vector<double>& weights);

  void clear();

  const SolverLog& getLog() { return log_; }

  void setKnownInliers(const std::set<size_t>& indices) {
    known_inliers_ = indices;
  }

  void setCorruptedOdom(const std::set<size_t>& indices) {
    corrupted_odom_indices_ = indices;
  }

  void setIterationCallback(const IterationCallback& callback) {
    iter_callback_ = callback;
  }

 protected:
  std::shared_ptr<gtsam::NonlinearOptimizerParams> getBaseParams() const {
    auto base = config_.optimizer_params;
    base->verbosity =
        static_cast<OptimizerParams::Verbosity>(config_.verbosity);
    if (iter_callback_) {
      base->iterationHook = iter_callback_;
    }

    return base;
  }

  virtual gtsam::Values optimize(const gtsam::NonlinearFactorGraph& factors,
                                 const gtsam::Values& initial,
                                 std::vector<double>& weights) = 0;

 protected:
  SolverLog log_;
  SolverConfig config_;
  std::set<size_t> known_inliers_;
  std::set<size_t> corrupted_odom_indices_;
  std::function<void(size_t, double, double)> iter_callback_;
};

}  // namespace kimera_rpgo
