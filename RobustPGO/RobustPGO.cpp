/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/RobustPGO.h"

RobustPGO::RobustPGO(OutlierRemoval* OR,
                     int solvertype, 
                     std::vector<char> special_symbols) :
                     GenericSolver(solvertype, special_symbols), 
                     outlier_removal_(OR) {
}

void RobustPGO::optimize() {
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

void RobustPGO::force_optimize() {
  if (debug_) log<WARNING>("Forcing optimization, typically should only use update method. ");
  optimize();
}

void RobustPGO::update(gtsam::NonlinearFactorGraph nfg, 
                       gtsam::Values values, 
                       gtsam::FactorIndices factorsToRemove) {
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