#pragma once

#include "kimera_rpgo/solver/solver.h"

namespace kimera_rpgo {

class LeastSquaresSolver : public Solver {
 public:
  using OptimizerPtr = std::unique_ptr<gtsam::NonlinearOptimizer>;
  using OptimizerFactory =
      std::function<OptimizerPtr(const gtsam::NonlinearFactorGraph&,
                                 const gtsam::Values&)>;

  explicit LeastSquaresSolver(const SolverConfig& config);
  ~LeastSquaresSolver();

 private:
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& factors,
                         const gtsam::Values& initial,
                         std::vector<double>& weights) override;

 private:
  const SolverConfig config_;
  OptimizerFactory optimizer_factory_;
};

}  // namespace kimera_rpgo
