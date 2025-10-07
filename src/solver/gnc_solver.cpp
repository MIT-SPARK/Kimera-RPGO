#include "kimera_rpgo/solver/gnc_solver.h"

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace kimera_rpgo {

using gtsam::GaussNewtonParams;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

using IndexVector = gtsam::FastVector<size_t>;

template <typename InnerParams>
using GncOpt = gtsam::GncOptimizer<gtsam::GncParams<InnerParams>>;

template <typename InnerParams>
GncOpt<InnerParams> makeOptimizer(const OptimizerParams* base,
                                  const ::kimera_rpgo::GncParams& config,
                                  const NonlinearFactorGraph& factors,
                                  const Values& initial,
                                  const IndexVector& inliers) {
  using Params = gtsam::GncParams<InnerParams>;
  const auto derived = dynamic_cast<const InnerParams*>(base);
  if (!derived) {
    throw std::invalid_argument("Optimizer option and param mismatch");
  }

  Params params(*derived);
  params.setMaxIterations(config.max_iterations);
  params.setMuStep(config.mu_step);
  switch (config.robust_cost) {
    case LossType::TLS:
      params.lossType = gtsam::TLS;
      break;
    case LossType::GM:
      params.lossType = gtsam::GM;
      break;
    default:
      std::invalid_argument("GNC only supports TLS and GM");
      break;
  }

  if (!inliers.empty()) {
    params.setKnownInliers(inliers);
  }

  GncOpt<InnerParams> optimizer(factors, initial, params);
  if (config.barc_sq > 0) {
    optimizer.setInlierCostThresholds(config.barc_sq);
  } else {
    optimizer.setInlierCostThresholdsAtProbability(config.inlier_probability);
  }

  return optimizer;
}

GncSolver::GncSolver(const SolverConfig& config)
    : Solver(config), config_(config) {
  if (!config.gnc_params) {
    throw std::runtime_error("GNC Params not set...");
  }
}

GncSolver::~GncSolver() {}

gtsam::Values GncSolver::optimize(const FactorGraph& factors,
                                  const Values& initial,
                                  std::vector<double>& weights) {
  const auto base = getBaseParams();
  const auto inliers = findInliers(factors);
  switch (config_.least_squares_option) {
    case SolverConfig::LeastSquaresOption::GN: {
      auto optimizer = makeOptimizer<GaussNewtonParams>(
          base.get(), *config_.gnc_params, factors, initial, inliers);

      auto result = optimizer.optimize();
      auto vec_weights = optimizer.getWeights();
      weights = std::vector<double>(vec_weights.data(),
                                    vec_weights.data() + vec_weights.size());

      return result;
    }
    case SolverConfig::LeastSquaresOption::LM: {
      auto optimizer = makeOptimizer<LevenbergMarquardtParams>(
          base.get(), *config_.gnc_params, factors, initial, inliers);
      auto result = optimizer.optimize();
      auto vec_weights = optimizer.getWeights();
      weights = std::vector<double>(vec_weights.data(),
                                    vec_weights.data() + vec_weights.size());

      return result;
    }
    default:
      throw std::invalid_argument("Unexpected Least Squares option for GNC");
  }
}

IndexVector GncSolver::findInliers(const FactorGraph& factors) const {
  IndexVector inlier_indices;
  for (size_t idx = 0; idx < factors.size(); idx++) {
    if (known_inliers_.count(idx)) {
      inlier_indices.push_back(idx);
      continue;
    }

    if (!config_.gnc_params->fix_odom) {
      continue;
    }

    if (corrupted_odom_indices_.count(idx)) {
      continue;
    }

    // TODO(Yun) check if between factor?
    auto factor = factors.at(idx);
    if (factor->keys().size() == 2 && factor->back() == factor->front() + 1) {
      inlier_indices.push_back(idx);
    }
  }

  return inlier_indices;
}

}  // namespace kimera_rpgo
