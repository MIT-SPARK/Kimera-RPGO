/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include "RobustPGO/RobustPGO.h"
#include "RobustPGO/graph_utils/graph_utils.h" 

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

template<class T>
void RobustPGO::load_graph(gtsam::GraphAndValues gv) {
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // first order the graph 
  graph_utils::orderGraph<T>(nfg);

  // for now not adding prior in here
  gtsam::Key current_key = nfg[0]->front();
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

      if (debug_) {
        std::cout << "odometry: " << front << ">" << back << std::endl; 
        nfg[i]->print();
      }

      update(new_factors, new_values);
      current_key++; 
    } else { 
      // loop closure factor 
      gtsam::NonlinearFactorGraph new_factors; 
      new_factors.add(nfg[i]);

      if (debug_) {
        std::cout << "loop closure: " << front << ">" << back << std::endl;
        nfg[i]->print(); 
      }

      update(new_factors, gtsam::Values());
    }
  }
}
