/*
Robust solver class
author: Yun Chang, Luca Carlone
*/

#include "KimeraRPGO/RobustSolver.h"

#include <chrono>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/logger.h"
#include "KimeraRPGO/outlier/pcm.h"
#include "KimeraRPGO/utils/type_utils.h"

namespace KimeraRPGO {

typedef std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GraphAndValues;

RobustSolver::RobustSolver(const RobustSolverParams& params)
    : GenericSolver(params.solver, params.specialSymbols),
      params_(params),
      gnc_num_inliers_(0) {
  switch (params.outlierRemovalMethod) {
    case OutlierRemovalMethod::NONE: {
      outlier_removal_ =
          nullptr;  // only returns optimize true or optimize false
    } break;
    case OutlierRemovalMethod::PCM2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm2D>(params.pcm_odomThreshold,
                                         params.pcm_lcThreshold,
                                         params.incremental,
                                         params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<Pcm3D>(params.pcm_odomThreshold,
                                         params.pcm_lcThreshold,
                                         params.incremental,
                                         params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM_Simple2D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple2D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.incremental,
                                               params.specialSymbols);
    } break;
    case OutlierRemovalMethod::PCM_Simple3D: {
      outlier_removal_ =
          KimeraRPGO::make_unique<PcmSimple3D>(params.pcmDist_transThreshold,
                                               params.pcmDist_rotThreshold,
                                               params.incremental,
                                               params.specialSymbols);
    } break;
    default: {
      log<WARNING>("Undefined outlier removal method");
      exit(EXIT_FAILURE);
    }
  }

  // toggle verbosity
  switch (params.verbosity) {
    case Verbosity::UPDATE: {
      if (outlier_removal_) outlier_removal_->setQuiet();
    } break;
    case Verbosity::QUIET: {
      if (outlier_removal_)
        outlier_removal_->setQuiet();  // set outlier removal quiet
      setQuiet();                      // set solver quiet
    } break;
    case Verbosity::VERBOSE: {
      log<INFO>("Starting RobustSolver.");
    } break;
    default: {
      log<WARNING>("Unrecognized verbosity. Automatically setting to UPDATE. ");
    }
  }

  // set log output
  if (params.log_output) {
    if (outlier_removal_) outlier_removal_->logOutput(params.log_folder);
    log_ = true;
    log_folder_ = params.log_folder;
    std::string filename = log_folder_ + "/rpgo_status.csv";
    std::ofstream outfile;
    outfile.open(filename);
    outfile << "graph-size,spin-time(mu-s)\n";
    outfile.close();
  }
}

void RobustSolver::optimize() {
  if (solver_type_ == Solver::LM) {
    gtsam::LevenbergMarquardtParams lmParams;
    lmParams.diagonalDamping = true;
    if (debug_) {
      lmParams.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM");
    }
    if (params_.use_gnc_ && outlier_removal_) {
      size_t num_odom_factors = outlier_removal_->getNumOdomFactors();
      size_t num_special_factors = outlier_removal_->getNumSpecialFactors();
      gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams(lmParams);
      // Set odometry and special factors as known inliers
      std::vector<size_t> known_inlier_factor_indices(num_odom_factors +
                                                      num_special_factors);
      std::iota(std::begin(known_inlier_factor_indices),
                std::end(known_inlier_factor_indices),
                0);
      gncParams.setKnownInliers(known_inlier_factor_indices);
      // Create GNC optimizer
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams> >
          gnc_optimizer(nfg_, values_, gncParams);
      switch (params_.gnc_threshold_mode_) {
        case (params_.GncThresholdMode::COST):
          gnc_optimizer.setInlierCostThresholds(params_.gnc_inlier_threshold_);
          break;
        case (params_.GncThresholdMode::PROBABILITY):
          gnc_optimizer.setInlierCostThresholdsAtProbability(
              params_.gnc_inlier_threshold_);
          break;
        default:
          log<WARNING>("Unsupported GNC threshold mode. ");
      }
      // Optimize and get weights
      auto opt_start_t = std::chrono::high_resolution_clock::now();
      values_ = gnc_optimizer.optimize();
      gnc_weights_ = gnc_optimizer.getWeights();
      gnc_num_inliers_ = static_cast<size_t>(gnc_weights_.sum()) -
                         known_inlier_factor_indices.size();
      auto opt_stop_t = std::chrono::high_resolution_clock::now();
      auto opt_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          opt_stop_t - opt_start_t);
      if (debug_) {
        log<INFO>(
            "GNC optimize took %1% milliseconds. %2% loop closures with "
            "%3% inliers. ") %
            opt_duration.count() % outlier_removal_->getNumLCInliers() %
            gnc_num_inliers_;
      }
    } else {
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, lmParams)
                    .optimize();
    }
  } else if (solver_type_ == Solver::GN) {
    gtsam::GaussNewtonParams gnParams;
    if (debug_) {
      gnParams.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    if (params_.use_gnc_ && outlier_removal_) {
      size_t num_odom_factors = outlier_removal_->getNumOdomFactors();
      size_t num_special_factors = outlier_removal_->getNumSpecialFactors();
      gtsam::GncParams<gtsam::GaussNewtonParams> gncParams(gnParams);
      // Set odometry and special factors as known inliers
      std::vector<size_t> known_inlier_factor_indices(num_odom_factors +
                                                      num_special_factors);
      std::iota(std::begin(known_inlier_factor_indices),
                std::end(known_inlier_factor_indices),
                0);
      gncParams.setKnownInliers(known_inlier_factor_indices);
      // Create GNC optimizer
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams> >
          gnc_optimizer(nfg_, values_, gncParams);
      switch (params_.gnc_threshold_mode_) {
        case (params_.GncThresholdMode::COST):
          gnc_optimizer.setInlierCostThresholds(params_.gnc_inlier_threshold_);
          break;
        case (params_.GncThresholdMode::PROBABILITY):
          gnc_optimizer.setInlierCostThresholdsAtProbability(
              params_.gnc_inlier_threshold_);
          break;
        default:
          log<WARNING>("Unsupported GNC threshold mode. ");
      }
      // Optimize and get weights
      auto opt_start_t = std::chrono::high_resolution_clock::now();
      values_ = gnc_optimizer.optimize();
      gnc_weights_ = gnc_optimizer.getWeights();
      gnc_num_inliers_ = static_cast<size_t>(gnc_weights_.sum()) -
                         known_inlier_factor_indices.size();
      auto opt_stop_t = std::chrono::high_resolution_clock::now();
      auto opt_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          opt_stop_t - opt_start_t);
      if (debug_) {
        log<INFO>(
            "GNC optimize took %1% milliseconds. %2% loop closures with "
            "%3% inliers. ") %
            opt_duration.count() % outlier_removal_->getNumLCInliers() %
            gnc_num_inliers_;
      }
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, gnParams).optimize();
  } else {
    log<WARNING>("Unsupported Solver");
    exit(EXIT_FAILURE);
  }
}

void RobustSolver::forceUpdate(const gtsam::NonlinearFactorGraph& nfg,
                               const gtsam::Values& values) {
  if (outlier_removal_) {
    outlier_removal_->removeOutliers(nfg, values, &nfg_, &values_);
  } else {
    addAndCheckIfOptimize(nfg, values);
  }
  // optimize
  optimize();
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& factors,
                          const gtsam::Values& values,
                          bool optimize_graph) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  bool do_optimize;
  if (outlier_removal_) {
    do_optimize =
        outlier_removal_->removeOutliers(factors, values, &nfg_, &values_);
  } else {
    do_optimize = addAndCheckIfOptimize(factors, values);
  }

  if (do_optimize & optimize_graph) optimize();  // optimize once after loading

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_time =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  // Log status
  if (log_) {
    std::string filename = log_folder_ + "/rpgo_status.csv";
    std::ofstream outfile;
    outfile.open(filename, std::ofstream::out | std::ofstream::app);
    outfile << nfg_.size() << "," << spin_time.count() << std::endl;
    outfile.close();
  }
  return;
}

void RobustSolver::removePriorFactorsWithPrefix(const char& prefix,
                                                bool optimize_graph) {
  if (outlier_removal_) {
    // removing loop closure so values should not change
    outlier_removal_->removePriorFactorsWithPrefix(prefix, &nfg_);
  } else {
    removePriorsWithPrefix(prefix);
  }
  if (optimize_graph) optimize();
  return;
}

void RobustSolver::removeLastLoopClosure(char prefix_1, char prefix_2) {
  ObservationId id(prefix_1, prefix_2);
  if (outlier_removal_) {
    // removing loop closure so values should not change
    outlier_removal_->removeLastLoopClosure(id, &nfg_);
  } else {
    removeLastFactor();
  }

  optimize();
  return;
}

void RobustSolver::saveData(std::string folder_path) const {
  std::string g2o_file_path = folder_path + "/result.g2o";
  gtsam::writeG2o(nfg_, values_, g2o_file_path);
  if (outlier_removal_) {
    outlier_removal_->saveData(folder_path);
  }
}

}  // namespace KimeraRPGO
