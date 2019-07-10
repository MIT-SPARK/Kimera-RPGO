/* 
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
*/

#ifndef ROBUSTPGO_H
#define ROBUSTPGO_H

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "RobustPGO/GenericSolver.h"
#include "RobustPGO/logger.h"
#include "RobustPGO/OutlierRemoval.h"

class RobustPGO : public GenericSolver{
public:
  RobustPGO(OutlierRemoval* OR,
            int solvertype=1, 
            std::vector<char> special_symbols=std::vector<char>());
      // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton

  void update(gtsam::NonlinearFactorGraph nfg=gtsam::NonlinearFactorGraph(), 
              gtsam::Values values=gtsam::Values(),
              gtsam::FactorIndices factorsToRemove=gtsam::FactorIndices());

  void forceUpdate(gtsam::NonlinearFactorGraph nfg=gtsam::NonlinearFactorGraph(), 
              gtsam::Values values=gtsam::Values(),
              gtsam::FactorIndices factorsToRemove=gtsam::FactorIndices());

  void force_optimize();

  template<class T>
  void loadGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values,
      gtsam::PriorFactor<T> prior) {
    gtsam::NonlinearFactorGraph prior_factor;
    gtsam::Values prior_values; 
    prior_factor.add(prior);
    prior_values.insert(prior.key(), prior.prior());
    outlier_removal_->process(prior_factor, prior_values, nfg_, values_);

    connectGraph<T>(factors, values, prior.key());
  }

  template<class T>
  void addGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values,
      gtsam::BetweenFactor<T> connector) {

    gtsam::Key key0 = connector.back();

    gtsam::NonlinearFactorGraph connect_factor;
    gtsam::Values connect_values; 
    connect_factor.add(connector);
    connect_values.insert(key0, values.at<T>(key0));
    outlier_removal_->process(connect_factor, connect_values, nfg_, values_);

    connectGraph<T>(factors, values, key0);
  }

  template<class T>
  void connectGraph(gtsam::NonlinearFactorGraph factors, gtsam::Values values, gtsam::Key key0) {

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
        if (factors[i] != NULL &&
            factors[i]->front() == current_key && factors[i]->back() == current_key + 1) {
          end_of_odom = false;
          // if (debug_) {
          //     std::cout << "odometry: " << current_key << ">" << current_key + 1 << std::endl; 
          // }
          gtsam::Values new_values;
          gtsam::NonlinearFactorGraph new_factors; 
          new_values.insert(current_key + 1, values.at<T>(current_key + 1));
          new_factors.add(factors[i]);
          outlier_removal_->process(new_factors, new_values, nfg_, values_);
          current_key = current_key + 1;
          factors[i].reset();
          break;
        }
      }
      if (end_of_odom) extracted_odom = true; 
    }

    // now search for the special symbols (i.e. artifacts)
    for (size_t i = 0; i < factors.size(); i++) { 
      if (factors[i] != NULL){
        gtsam::Symbol symb(factors[i]->back());
        if (specialSymbol(symb.chr())) {
          gtsam::Values new_values;
          gtsam::NonlinearFactorGraph new_factors; 
          new_values.insert(factors[i]->back(), values.at<T>(factors[i]->back()));
          new_factors.add(factors[i]);

          outlier_removal_->process(new_factors, new_values, nfg_, values_);
          factors[i].reset();
          break;
        }
      }
    }
     
    // add the other non-odom loop closures
    for (size_t i = 0; i < factors.size(); i++) { 
      if (factors[i] != NULL) {
        gtsam::NonlinearFactorGraph new_factors;
        // if (debug_) {
        //   std::cout << "loop closure: " << factors[i]->front() << ">" << factors[i]->back() << std::endl;
        // }
        new_factors.add(factors[i]);
        outlier_removal_->process(new_factors, gtsam::Values(), nfg_, values_);
      }
    }
  }

private:
  OutlierRemoval* outlier_removal_; // outlier removal method; 

  void optimize(); 
};

#endif