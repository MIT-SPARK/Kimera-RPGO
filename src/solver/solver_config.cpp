#include "kimera_rpgo/solver/solver_config.h"

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <stdexcept>

namespace kimera_rpgo {

void SolverConfig::setLeastSquaresParams(
    std::shared_ptr<OptimizerParams> params) {
  // Try cast to GN, LM, or Dogleg
  auto gn_params = std::dynamic_pointer_cast<GaussNewtonParams>(params);
  if (gn_params) {
    least_squares_option = SolverConfig::LeastSquaresOption::GN;
    optimizer_params = params;
    return;
  }

  auto lm_params = std::dynamic_pointer_cast<LevenbergMarquardtParams>(params);
  if (lm_params) {
    least_squares_option = SolverConfig::LeastSquaresOption::LM;
    optimizer_params = params;
    return;
  }

  auto dogleg_params = std::dynamic_pointer_cast<DoglegParams>(params);
  if (dogleg_params) {
    least_squares_option = SolverConfig::LeastSquaresOption::DOGLEG;
    optimizer_params = params;
    return;
  }

  throw std::invalid_argument("Unexpected optimizer params");
}

void SolverConfig::setGradientParams(std::shared_ptr<OptimizerParams> params) {
  // Currently only supports conjugate gradient descent
  gradient_option = SolverConfig::GradientOption::CONJUGATE;
  optimizer_params = params;
}

void SolverConfig::setLeastSquaresParamsDefault() {
  switch (least_squares_option) {
    case SolverConfig::LeastSquaresOption::GN:
      optimizer_params = std::make_shared<GaussNewtonParams>();
      break;
    case SolverConfig::LeastSquaresOption::LM:
      optimizer_params = std::make_shared<LevenbergMarquardtParams>();
      break;
    case SolverConfig::LeastSquaresOption::DOGLEG:
      optimizer_params = std::make_shared<DoglegParams>();
      break;
    default:
      throw std::invalid_argument("Unexpected least square solver type");
      break;
  }
}

void SolverConfig::setGradientParamsDefault() {
  switch (gradient_option) {
    case SolverConfig::GradientOption::GRADIENT:
      // GRADIENT option not exposed as of Jan 2025
      // https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h#L200
      throw std::invalid_argument("Only conjugate gradient supported");
      break;
    case SolverConfig::GradientOption::CONJUGATE:
      optimizer_params = std::make_shared<OptimizerParams>();
      break;
    default:
      throw std::invalid_argument("Unexpected gradient solver type");
      break;
  }

  // Increase max iterations
  optimizer_params->maxIterations = 1000;
}

void SolverConfig::setGncParamsDefault() {
  gnc_params = std::make_shared<GncParams>();
}

void SolverConfig::setGncParams(const GncParams& params) {
  gnc_params = std::make_shared<GncParams>(params);
}

std::ostream& operator<<(std::ostream& os,
                         const SolverConfig::LeastSquaresOption& option) {
  switch (option) {
    case SolverConfig::LeastSquaresOption::GN:
      os << "Gauss-Newton";
      break;
    case SolverConfig::LeastSquaresOption::LM:
      os << "Levenberg-Marquardt";
      break;
    case SolverConfig::LeastSquaresOption::DOGLEG:
      os << "Dogleg";
      break;
  }

  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const SolverConfig::GradientOption& option) {
  switch (option) {
    case SolverConfig::GradientOption::GRADIENT:
      os << "Gradient-Descent";
      break;
    case SolverConfig::GradientOption::CONJUGATE:
      os << "Conjugate-Gradient-Descent";
      break;
  }

  return os;
}

std::ostream& operator<<(std::ostream& os, const OptimizerParams& params) {
  os << "OptimizerParams:\n";
  os << "  maxIterations: " << params.maxIterations << "\n";
  os << "  relativeErrorTol: " << params.relativeErrorTol << "\n";
  os << "  absoluteErrorTol: " << params.absoluteErrorTol << "\n";
  os << "  errorTol: " << params.errorTol << "\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const GncParams& params) {
  os << "GncParams:\n";
  if (params.barc_sq > 0) {
    os << "  barc_sq: " << params.barc_sq << "\n";
  } else {
    os << "  inlier_probability: " << params.inlier_probability << "\n";
  }

  os << "  max_iterations: " << params.max_iterations << "\n";
  os << "  mu_step: " << params.mu_step << "\n";
  os << "  robust_cost: " << params.robust_cost << "\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const SolverConfig& config) {
  os << "least_squares_option: " << config.least_squares_option << "\n";
  os << "gradient_option: " << config.gradient_option << "\n";
  os << "optimizer_params: \n" << config.optimizer_params << "\n";
  os << "gnc_params: \n" << config.gnc_params << "\n";
  return os;
}
}  // namespace kimera_rpgo
