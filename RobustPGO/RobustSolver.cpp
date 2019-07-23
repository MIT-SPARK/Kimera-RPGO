/*
Generic solver class
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/RobustSolver.h"

namespace RobustPGO {

RobustSolver::RobustSolver(const RobustSolverParams& params) :
    GenericSolver(params.solver, params.specialSymbols) {
  switch (params.outlierRemovalMethod) {
    case OutlierRemovalMethod::NONE :
    {
      outlier_removal_ = nullptr; // only returns optimize true or optimize false
    }
    break;
    case OutlierRemovalMethod::PCM2D :
    {
      outlier_removal_ = std::make_unique<Pcm2D>(
          params.pcm_odomThreshold, params.pcm_lcThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM3D :
    {
      outlier_removal_ = std::make_unique<Pcm3D>(
          params.pcm_odomThreshold, params.pcm_lcThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM_Simple2D:
    {
      outlier_removal_ = std::make_unique<PcmSimple2D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold, params.specialSymbols);
    }
    break;
    case OutlierRemovalMethod::PCM_Simple3D:
    {
      outlier_removal_ = std::make_unique<PcmSimple3D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold, params.specialSymbols);
    }
    break;
    default:
    {
      log<WARNING>("Undefined outlier removal method");
      exit (EXIT_FAILURE);
    }
  }

  // toggle verbosity
  switch (params.verbosity) {
    case Verbosity::UPDATE :
    {
      if (outlier_removal_) outlier_removal_->setQuiet();
    }
    break;
    case Verbosity::QUIET :
    {
      if (outlier_removal_) outlier_removal_->setQuiet(); // set outlier removal quiet
      setQuiet(); // set solver quiet
    }
    break;
    case Verbosity::VERBOSE :
    {
      log<INFO>("Starting RobustSolver.");
    }
    break;
    default:
    {
      log<WARNING>("Unrecognized verbosity. Automatically setting to UPDATE. ");
    }
  }
}

void RobustSolver::optimize() {
  if (solver_type_ == Solver::LM) {
    gtsam::LevenbergMarquardtParams params;
    if (debug_){
      params.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM");
    }
    params.diagonalDamping = true;
    values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
  }else if (solver_type_ == Solver::GN) {
    gtsam::GaussNewtonParams params;
    if (debug_) {
      params.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
  }else {
      log<WARNING>("Unsupported Solver");
      exit (EXIT_FAILURE);
  }
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values) {

  // loop closures/outlier rejection
  bool process_lc;
  if (outlier_removal_) {
    process_lc = outlier_removal_->removeOutliers(nfg, values, nfg_, values_);
  } else {
    process_lc = addAndCheckIfOptimize(nfg, values); // use default process
  }

  // optimize
  if (process_lc) {
    optimize();
  }
}

void RobustSolver::forceUpdate(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values) {

  if (outlier_removal_) {
    outlier_removal_->removeOutliers(nfg, values, nfg_, values_);
  } else {
    addAndCheckIfOptimize(nfg, values);
  }
  // optimize
  optimize();
}

void RobustSolver::addOdometry(const gtsam::NonlinearFactorGraph& odom_factor, const gtsam::Values& odom_values) {
  // TODO add warning if more tha one factor / value
  if (outlier_removal_) {
    outlier_removal_->removeOutliers(odom_factor, odom_values, nfg_, values_);
  } else {
    addAndCheckIfOptimize(odom_factor, odom_values);
  }
}

}
