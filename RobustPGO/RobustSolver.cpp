/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/RobustSolver.h"

namespace RobustPGO {

RobustSolver::RobustSolver(const std::shared_ptr<OutlierRemoval>& outlier_remover,
                     int solvertype, 
                     const std::vector<char>& special_symbols) :
                     GenericSolver(solvertype, special_symbols), 
                     outlier_removal_(outlier_remover) {
}

void RobustSolver::optimize() {
  if (solver_type_ == 1) {
    gtsam::LevenbergMarquardtParams params;
    if (debug_){
      params.setVerbosityLM("SUMMARY");
      log<INFO>("Running LM"); 
    }
    params.diagonalDamping = true; 
    values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
  }else if (solver_type_ == 2) {
    gtsam::GaussNewtonParams params;
    if (debug_) {
      params.setVerbosity("ERROR");
      log<INFO>("Running GN");
    }
    values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
  }else if (solver_type_ == 3) {
    // something
  }
  
  // save result
  if (save_g2o_) {
    gtsam::writeG2o(nfg_, values_, g2o_file_path_);
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
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  bool do_optimize = outlier_removal_->process(nfg, values, nfg_, values_);
  // optimize
  if (do_optimize) {
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

  outlier_removal_->processForcedLoopclosure(nfg, values, nfg_, values_);
  // optimize
  optimize();
}

}