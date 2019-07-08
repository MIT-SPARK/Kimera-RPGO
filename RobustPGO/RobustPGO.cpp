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

// TODO add another loadGraph that doesn't require the prior
template<class T>
void RobustPGO::loadGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values,
    gtsam::PriorFactor<T> prior) {
  gtsam::NonlinearFactorGraph prior_factor;
  prior_factor.add(prior);
  update(prior_factor, gtsam::Values()); // triggers initialization
  addGraph(factors, values, prior.key());
}

template<class T>
void RobustPGO::addGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values, gtsam::Key key0) {
  // load graph assumes that the previous graph has been cleared
  gtsam::Key current_key = key0; // initial key
  // note that as of now only deal with between factors)
  // first load the odometry
  // order a nonlinear factor graph as odometry first 
  bool extracted_odom = false; 
  while (!extracted_odom) {
    bool end_of_odom = true; 
    for (size_t i = 0; i < factors.size(); i++) { 
      // search through 
      if (factors[i]->front() == current_key && factors[i]->back() == current_key + 1) {
        end_of_odom = false;
        gtsam::Values new_values;
        gtsam::NonlinearFactorGraph new_factors; 
        new_values.insert(current_key + 1, values.at<T>(current_key + 1));
        new_factors.add(factors[i]);

        update(new_factors, new_values);
        current_key = current_key + 1;
        factors[i].reset();
        break;
      }
    }
    if (end_of_odom) extracted_odom = true; 
  }

  // now search for the special symbols (i.e. artifacts)
  for (size_t i = 0; i < factors.size(); i++) { 
    gtsam::Symbol symb(factors[i]->back());
    if (specialSymbol(symb.chr())) {
      gtsam::Values new_values;
      gtsam::NonlinearFactorGraph new_factors; 
      new_values.insert(factors[i]->back(), values.at<T>(factors[i]->back()));
      new_factors.add(factors[i]);

      update(new_factors, new_values);
      factors[i].reset();
      break;
    }
  }
   
  // add the other non-odom loop closures
  for (size_t i = 0; i < factors.size(); i++) { 
    if (factors[i] != NULL) {
      gtsam::NonlinearFactorGraph new_factors;
      new_factors.add(factors[i]);
      update(new_factors, gtsam::Values());
    }
  }
}