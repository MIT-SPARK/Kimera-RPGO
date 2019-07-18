/* 
Generic solver class 
No outlier removal in this class 
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/GenericSolver.h"

namespace RobustPGO {

GenericSolver::GenericSolver(Solver solvertype, 
                             std::vector<char> special_symbols): 
  nfg_(gtsam::NonlinearFactorGraph()),
  values_(gtsam::Values()),
  solver_type_(solvertype),
  special_symbols_(special_symbols),
  debug_(true), 
  save_g2o_(false) {}

bool GenericSolver::specialSymbol(char symb) {
  for (size_t i = 0; i < special_symbols_.size(); i++) {
    if (special_symbols_[i] == symb) return true;
  }
  return false; 
}

void GenericSolver::update(const gtsam::NonlinearFactorGraph& nfg, 
                           const gtsam::Values& values, 
                           const gtsam::FactorIndices& factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  // add new values and factors
  nfg_.add(nfg);
  values_.insert(values);
  bool do_optimize = true; 

  // Do not optimize for just odometry additions 
  // odometry values would not have prefix 'l' unlike artifact values
  if (nfg.size() == 1 && values.size() == 1) {do_optimize = false;}

  // nothing added so no optimization 
  if (nfg.size() == 0 && values.size() == 0) {do_optimize = false;}

  if (factorsToRemove.size() > 0) 
    do_optimize = true;

  if (do_optimize) {
    // optimize
    if (solver_type_ == LM) {
      gtsam::LevenbergMarquardtParams params;
      if (debug_) {
        params.setVerbosityLM("SUMMARY");
        log<INFO>("Running LM"); 
      }
      params.diagonalDamping = true; 
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == GN) {
      gtsam::GaussNewtonParams params;
      if (debug_){
        params.setVerbosity("ERROR");
        log<INFO>("Running GN");
      }
      values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
    } else {
      log<WARNING>("Unsupported Solver");
      exit (EXIT_FAILURE);
    }
    // save result 
    if (save_g2o_) {
      gtsam::writeG2o(nfg_, values_, g2o_file_path_);
    }
  }
}

void GenericSolver::removeFactorsNoUpdate(
    gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }
}

}
