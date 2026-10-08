#pragma once

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <memory>
#include <ostream>

#include "kimera_rpgo/utils/types.h"

namespace kimera_rpgo {
using OptimizerParams = gtsam::NonlinearOptimizerParams;
using GaussNewtonParams = gtsam::GaussNewtonParams;
using LevenbergMarquardtParams = gtsam::LevenbergMarquardtParams;
using DoglegParams = gtsam::DoglegParams;

struct GncParams {
  using Ptr = std::shared_ptr<GncParams>;
  double barc_sq = 0;
  // If barc_sq is not set to valid value, default to using inlier probability
  double inlier_probability = 0.7;
  double mu_step = 1.4;
  size_t max_iterations = 100;
  bool fix_odom = true;
  LossType robust_cost = LossType::TLS;
};

struct SolverConfig {
  int verbosity = 0;
  enum class LeastSquaresOption {
    GN,
    LM,
    DOGLEG
  } least_squares_option = LeastSquaresOption::LM;

  enum class GradientOption {
    GRADIENT,
    CONJUGATE
  } gradient_option = GradientOption::CONJUGATE;

  std::shared_ptr<OptimizerParams> optimizer_params;

  GncParams::Ptr gnc_params;

  void setLeastSquaresParams(std::shared_ptr<OptimizerParams> params);
  void setLeastSquaresParamsDefault();
  void setGradientParams(std::shared_ptr<OptimizerParams> params);
  void setGradientParamsDefault();

  void setGncParamsDefault();
  void setGncParams(const GncParams& gnc_params);
};

std::ostream& operator<<(std::ostream& os,
                         const SolverConfig::LeastSquaresOption& option);

std::ostream& operator<<(std::ostream& os,
                         const SolverConfig::GradientOption& option);

std::ostream& operator<<(std::ostream& os, const OptimizerParams& params);

std::ostream& operator<<(std::ostream& os, const GncParams& params);

std::ostream& operator<<(std::ostream& os, const SolverConfig& config);
}  // namespace kimera_rpgo
