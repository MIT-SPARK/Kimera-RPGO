/* 
Generic solver class 
No outlier removal in this class 
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/GenericSolver.h"

GenericSolver::GenericSolver(int solvertype, 
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

void GenericSolver::update(gtsam::NonlinearFactorGraph nfg, 
                           gtsam::Values values, 
                           gtsam::FactorIndices factorsToRemove) {
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
  if (nfg.size() == 1 && values.size() == 1) {
    const gtsam::Symbol symb(values.keys()[0]); 
    if (!specialSymbol(symb.chr())) {do_optimize = false;}
  }

  // nothing added so no optimization 
  if (nfg.size() == 0 && values.size() == 0) {do_optimize = false;}

  if (factorsToRemove.size() > 0) 
    do_optimize = true;

  if (do_optimize) {
    // optimize
    if (solver_type_ == 1) {
      gtsam::LevenbergMarquardtParams params;
      if (debug_) {
        params.setVerbosityLM("SUMMARY");
        log<INFO>("Running LM"); 
      }
      params.diagonalDamping = true; 
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 2) {
      gtsam::GaussNewtonParams params;
      if (debug_){
        params.setVerbosity("ERROR");
        log<INFO>("Running GN");
      }
      values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 3) {
      // TODO: something (SE-SYNC?)
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

void GenericSolver::loadGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values) {
  // load graph assumes that the previous graph has been cleared
  gtsam::Key current_key = factors[0]->front();
  gtsam::Values init_values;
  init_values.insert(current_key, values.at<T>(current_key));
  update(gtsam::NonlinearFactorGraph(), init_values); // triggers initialization

  // For now assume that there are only odometry and loop closures 
  size_t num_factors = nfg.size(); 
  for (size_t i = 0; i < num_factors; i++) { 
    gtsam::Key front = nfg[i]->front();
    gtsam::Key back = nfg[i]->back();
    if (front == current_key && front == back - 1) { 
      // odometry factor 
      gtsam::Values new_values; 
      new_values.insert(back, values.at<T>(back));
      gtsam::NonlinearFactorGraph new_factors; 
      new_factors.add(nfg[i]);

      if (debug) {
        std::cout << "odometry: " << front << ">" << back << std::endl; 
        nfg[i]->print();
      }

      pgo->update(new_factors, new_values);
      current_key++; 
    } else { 
      // loop closure factor 
      gtsam::NonlinearFactorGraph new_factors; 
      new_factors.add(nfg[i]);

      if (debug) {
        std::cout << "loop closure: " << front << ">" << back << std::endl;
        nfg[i]->print(); 
      }

      pgo->update(new_factors, gtsam::Values());
    }
  }
}

void GenericSolver::addGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values);
