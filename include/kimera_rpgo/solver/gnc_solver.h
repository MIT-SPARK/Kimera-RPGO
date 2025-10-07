#pragma once

#include <gtsam/nonlinear/GncParams.h>

#include "kimera_rpgo/solver/solver.h"

namespace kimera_rpgo {

class GncSolver : public Solver {
 public:
  using FactorGraph = gtsam::NonlinearFactorGraph;
  using InlierIndices = gtsam::FastVector<size_t>;

  explicit GncSolver(const SolverConfig& config);
  ~GncSolver();

 private:
  gtsam::Values optimize(const FactorGraph& factors,
                         const gtsam::Values& initial,
                         std::vector<double>& weights) override;

  InlierIndices findInliers(const FactorGraph& graph) const;

  const SolverConfig config_;
};

}  // namespace kimera_rpgo
