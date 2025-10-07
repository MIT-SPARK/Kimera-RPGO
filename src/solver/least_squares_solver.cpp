#include "kimera_rpgo/solver/least_squares_solver.h"

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "kimera_rpgo/utils/utils.h"

namespace kimera_rpgo {

using gtsam::DoglegOptimizer;
using gtsam::DoglegParams;
using gtsam::GaussNewtonOptimizer;
using gtsam::GaussNewtonParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::LevenbergMarquardtParams;
using gtsam::NoiseModelFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

using gtsam::NonlinearOptimizer;

namespace {

template <typename Solver, typename Params>
std::unique_ptr<Solver> makeOpt(const OptimizerParams* base,
                                const NonlinearFactorGraph& factors,
                                const Values& initial) {
  auto derived = dynamic_cast<const Params*>(base);
  if (!derived) {
    throw std::runtime_error("Optimizer option and param mismatch");
  }

  return std::make_unique<Solver>(factors, initial, *derived);
}

}  // namespace

LeastSquaresSolver::LeastSquaresSolver(const SolverConfig& config)
    : Solver(config), config_(config) {
  auto base = config.optimizer_params;
  base->verbosity = static_cast<OptimizerParams::Verbosity>(config.verbosity);
  // TODO(nathan) setup logging callback

  switch (config.least_squares_option) {
    case SolverConfig::LeastSquaresOption::GN:
      optimizer_factory_ = [base](const auto& graph, const auto& init) {
        return makeOpt<GaussNewtonOptimizer, GaussNewtonParams>(
            base.get(), graph, init);
      };
      break;
    case SolverConfig::LeastSquaresOption::LM:
      optimizer_factory_ = [base](const auto& graph, const auto& init) {
        return makeOpt<LevenbergMarquardtOptimizer, LevenbergMarquardtParams>(
            base.get(), graph, init);
      };
      break;
    case SolverConfig::LeastSquaresOption::DOGLEG:
      optimizer_factory_ = [base](const auto& graph, const auto& init) {
        return makeOpt<DoglegOptimizer, DoglegParams>(base.get(), graph, init);
      };
      break;
    default:
      throw std::invalid_argument("Unexpected Least Squares option");
  }
}

LeastSquaresSolver::~LeastSquaresSolver() {}

gtsam::Values LeastSquaresSolver::optimize(const NonlinearFactorGraph& factors,
                                           const Values& initial,
                                           std::vector<double>& weights) {
  std::cout << "Least squares optimize\n";
  const auto optimizer = optimizer_factory_(factors, initial);
  if (!optimizer) {
    throw std::invalid_argument("Invalid optimizer config!");
  }

  const auto result = optimizer->optimize();

  weights.resize(factors.size());
  for (size_t i = 0; i < factors.size(); i++) {
    // TODO(Yun) handle factors not derived from NoiseModelFactor
    auto factor = factor_pointer_cast<NoiseModelFactor>(factors[i]);
    if (!factor) {
      throw std::runtime_error("factor not derived from NoiseModelFactor!");
    }

    weights[i] = factor->weight(result);
  }

  return result;
}

}  // namespace kimera_rpgo
