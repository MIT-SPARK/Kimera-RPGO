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
      outlier_removal_ = nullptr;
      // only returns optimize true or optimize false
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
    case OutlierRemovalMethod::PCM_Distance2D:
    {
      outlier_removal_ = std::make_unique<PcmDistance2D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold);
    }
    break;
    case OutlierRemovalMethod::PCM_Distance3D:
    {
      outlier_removal_ = std::make_unique<PcmDistance3D>(
          params.pcmDist_transThreshold, params.pcmDist_rotThreshold);
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
      if (outlier_removal_) outlier_removal_->setQuiet(); 
      // set outlier removal quiet
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

void RobustSolver::force_optimize() {
  if (debug_) log<WARNING>("Forcing optimization, typically should only use update method. ");
  optimize();
}

void RobustSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values,
                       const gtsam::FactorIndices& factorsToRemove) {
  // remove factors
  bool remove_factors = false;
  if (factorsToRemove.size() > 0) {remove_factors = true;}
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  // loop closures/outlier rejection 
  bool process_lc;
  if (outlier_removal_) {
    process_lc = outlier_removal_->process(nfg, values, nfg_, values_);
  } else {
    process_lc = process(nfg, values); // use default process
  }

  // optimize
  if (remove_factors || process_lc) {
    optimize();
  }
}

void RobustSolver::forceUpdate(const gtsam::NonlinearFactorGraph& nfg,
                       const gtsam::Values& values,
                       const gtsam::FactorIndices& factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  if (outlier_removal_) {
    outlier_removal_->processForcedLoopclosure(nfg, values, nfg_, values_);
  } else {
    process(nfg, values);
  }
  // optimize
  optimize();
}

}