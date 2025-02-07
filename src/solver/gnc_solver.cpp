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

GncSolver::GncSolver(const SolverConfig& config) : Solver(config), config_(config) {
  if (!config.gnc_params) {
    throw std::runtime_error("GNC Params not set...");
  }
}

GncSolver::~GncSolver() {}

gtsam::Values GncSolver::optimize(const NonlinearFactorGraph& factors,
                                  const Values& initial,
                                  std::vector<double>& weights) {
  const auto& verbosity = static_cast<OptimizerParams::Verbosity>(config_.verbosity);
  switch (config_.least_squares_option) {
    case SolverConfig::LeastSquaresOption::GN: {
      auto gn_params =
          std::dynamic_pointer_cast<GaussNewtonParams>(config_.optimizer_params);
      // Potentially can also just detect param type instead of setting option. TODO(Yun)
      if (!gn_params) {
        throw std::invalid_argument("Optimizer option and param mismatch");
      }
      gn_params->verbosity = verbosity;
      auto gnc_params = setupGncParams(*gn_params);
      if (config_.gnc_params->fix_odom) {
        const auto& inliers = findInlierOdomIndices(factors);
        gnc_params.setKnownInliers(inliers);
      }
      switch (config_.gnc_params->robust_cost) {
        case LossType::TLS:
          gnc_params.lossType = gtsam::TLS;
          break;
        case LossType::GM:
          gnc_params.lossType = gtsam::GM;
          break;
        default:
          std::invalid_argument("GNC only supports TLS and GM");
          break;
      }
      gnc_params.verbosity =
          static_cast<gtsam::GncParams<GaussNewtonParams>::Verbosity>(config_.verbosity);
      gtsam::GncOptimizer<gtsam::GncParams<GaussNewtonParams> > gnc_optimizer(
          factors, initial, gnc_params);
      if (config_.gnc_params->barc_sq > 0) {
        gnc_optimizer.setInlierCostThresholds(config_.gnc_params->barc_sq);
      } else {
        gnc_optimizer.setInlierCostThresholdsAtProbability(
            config_.gnc_params->inlier_probability);
      }
      auto result = gnc_optimizer.optimize();
      auto vec_weights = gnc_optimizer.getWeights();
      weights = std::vector<double>(vec_weights.data(),
                                    vec_weights.data() + vec_weights.size());

      return result;
    }
    case SolverConfig::LeastSquaresOption::LM: {
      auto lm_params =
          std::dynamic_pointer_cast<LevenbergMarquardtParams>(config_.optimizer_params);
      if (!lm_params) {
        throw std::invalid_argument("Optimizer option and param mismatch");
      }
      lm_params->verbosity = verbosity;
      lm_params->verbosityLM =
          static_cast<LevenbergMarquardtParams::VerbosityLM>(config_.verbosity);
      auto gnc_params = setupGncParams(*lm_params);
      if (config_.gnc_params->fix_odom) {
        const auto& inliers = findInlierOdomIndices(factors);
        gnc_params.setKnownInliers(inliers);
      }
      switch (config_.gnc_params->robust_cost) {
        case LossType::TLS:
          gnc_params.lossType = gtsam::TLS;
          break;
        case LossType::GM:
          gnc_params.lossType = gtsam::GM;
          break;
        default:
          std::invalid_argument("GNC only supports TLS and GM");
          break;
      }
      gnc_params.verbosity =
          static_cast<gtsam::GncParams<LevenbergMarquardtParams>::Verbosity>(
              config_.verbosity);
      gtsam::GncOptimizer<gtsam::GncParams<LevenbergMarquardtParams> > gnc_optimizer(
          factors, initial, gnc_params);
      if (config_.gnc_params->barc_sq > 0) {
        gnc_optimizer.setInlierCostThresholds(config_.gnc_params->barc_sq);
      } else {
        gnc_optimizer.setInlierCostThresholdsAtProbability(
            config_.gnc_params->inlier_probability);
      }
      auto result = gnc_optimizer.optimize();
      auto vec_weights = gnc_optimizer.getWeights();
      weights = std::vector<double>(vec_weights.data(),
                                    vec_weights.data() + vec_weights.size());

      return result;
    }
    default:
      throw std::invalid_argument("Unexpected Least Squares option for GNC");
      return gtsam::Values();
  }
}

gtsam::GncParams<GaussNewtonParams> GncSolver::setupGncParams(
    const GaussNewtonParams& gn_param) const {
  gtsam::GncParams<GaussNewtonParams> gnc_param(gn_param);
  gnc_param.setMaxIterations(config_.gnc_params->max_iterations);
  gnc_param.setMuStep(config_.gnc_params->mu_step);
  return gnc_param;
}

gtsam::GncParams<LevenbergMarquardtParams> GncSolver::setupGncParams(
    const LevenbergMarquardtParams& lm_param) const {
  gtsam::GncParams<LevenbergMarquardtParams> gnc_param(lm_param);
  gnc_param.setMaxIterations(config_.gnc_params->max_iterations);
  gnc_param.setMuStep(config_.gnc_params->mu_step);
  return gnc_param;
}

GncSolver::IndexVector GncSolver::findInlierOdomIndices(
    const NonlinearFactorGraph& factors) const {
  IndexVector inlier_indices;
  uint64_t idx = 0;
  for (const auto& factor : factors) {
    if (corrupted_odom_indices_.count(idx)) {
      continue;
    }

    if (factor->keys().size() == 2 && factor->back() == factor->front() + 1) {
      inlier_indices.push_back(idx);
    }
    idx++;
  }
  return inlier_indices;
}
}  // namespace kimera_rpgo
