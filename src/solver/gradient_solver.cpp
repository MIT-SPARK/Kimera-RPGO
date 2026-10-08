#include "kimera_rpgo/solver/gradient_solver.h"

#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>

#include "kimera_rpgo/utils/utils.h"

namespace kimera_rpgo {
using gtsam::NoiseModelFactor;
using gtsam::NonlinearConjugateGradientOptimizer;
using gtsam::NonlinearFactorGraph;
using gtsam::NonlinearOptimizerParams;  // Switch to conjugate gradient params
using gtsam::Values;

GradientSolver::GradientSolver(const SolverConfig& config)
    : Solver(config), config_(config) {}

GradientSolver::~GradientSolver() {}

gtsam::Values GradientSolver::optimize(const NonlinearFactorGraph& factors,
                                       const Values& initial,
                                       std::vector<double>& weights) {
  NonlinearOptimizerParams params(*config_.optimizer_params);
  params.verbosity = static_cast<OptimizerParams::Verbosity>(config_.verbosity);
  switch (config_.gradient_option) {
    case SolverConfig::GradientOption::GRADIENT:
      throw std::logic_error("Not yet exposed by GTSAM");
      // params.gradientDescent = true;
      break;
    case SolverConfig::GradientOption::CONJUGATE:
      // params.gradientDescent = false;
      break;
    default:
      throw std::invalid_argument("Unknwon gradient descent option");
      break;
  }

  NonlinearConjugateGradientOptimizer gradient_optimizer(
      factors, initial, params);
  auto result = gradient_optimizer.optimize();
  weights.resize(factors.size());
  for (size_t i = 0; i < factors.size(); i++) {
    // TODO(Yun) handle factors not derived from NoiseModelFactor
    auto factor = factor_pointer_cast<NoiseModelFactor>(factors[i]);
    weights[i] = factor->weight(result);
  }

  return result;
}
}  // namespace kimera_rpgo
