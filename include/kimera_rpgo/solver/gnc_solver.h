#pragma once

#include <gtsam/nonlinear/GncParams.h>

#include "kimera_rpgo/solver/solver.h"

namespace kimera_rpgo {

class GncSolver : public Solver {
  typedef gtsam::FastVector<uint64_t> IndexVector;

 public:
  GncSolver(const SolverConfig& config);
  ~GncSolver();

 private:
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& factors,
                         const gtsam::Values& initial,
                         std::vector<double>& weights) override;

  gtsam::GncParams<gtsam::GaussNewtonParams> setupGncParams(
      const gtsam::GaussNewtonParams& gn_param) const;

  gtsam::GncParams<gtsam::LevenbergMarquardtParams> setupGncParams(
      const gtsam::LevenbergMarquardtParams& lm_param) const;

  IndexVector findInlierIndices(
      const gtsam::NonlinearFactorGraph& factors) const;

 private:
  SolverConfig config_;
};
}  // namespace kimera_rpgo
