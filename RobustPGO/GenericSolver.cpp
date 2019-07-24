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
  debug_(true) {}

bool GenericSolver::isSpecialSymbol(char symb) const {
  for (size_t i = 0; i < special_symbols_.size(); i++) {
    if (special_symbols_[i] == symb) return true;
  }
  return false;
}

bool GenericSolver::addAndCheckIfOptimize(const gtsam::NonlinearFactorGraph& nfg,
      const gtsam::Values& values) {
  // add new values and factors
  nfg_.add(nfg);
  values_.insert(values);
  bool do_optimize = true;

  // Do not optimize for just odometry (between) additions
  if (nfg.size() == 1 && nfg[0]->keys().size() == 2 && values.size() == 1) {return false;}

  // nothing added so no optimization
  if (nfg.size() == 0 && values.size() == 0) {return false;}

  return true;
}

void GenericSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values,
                           const gtsam::FactorIndices& factorsToRemove) {
  // remove factors
  bool remove_factors = false;
  if (factorsToRemove.size() > 0) {remove_factors = true;}
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  bool process_lc = addAndCheckIfOptimize(nfg, values);

  if (process_lc || remove_factors) {
    // optimize
    if (solver_type_ == Solver::LM) {
      gtsam::LevenbergMarquardtParams params;
      if (debug_) {
        params.setVerbosityLM("SUMMARY");
        log<INFO>("Running LM");
      }
      params.diagonalDamping = true;
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == Solver::GN) {
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
