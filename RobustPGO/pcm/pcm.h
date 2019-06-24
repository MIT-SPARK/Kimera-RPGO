/* 
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
*/

#ifndef PCM_H
#define PCM_H

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR 

#include <fstream>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "RobustPGO/graph_utils/graph_utils_functions.h" 
#include "RobustPGO/logger.h"
#include "RobustPGO/OutlierRemoval.h"

template<class T>
class PCM : public OutlierRemoval{
public:
  PCM(double odom_threshold, double pc_threshold, 
      std::vector<char> special_symbols=std::vector<char>()); 
  // initialize with odometry detect threshold and pairwise consistency threshold

  bool process(gtsam::NonlinearFactorGraph new_factors, 
               gtsam::Values new_values,
               gtsam::NonlinearFactorGraph& output_nfg, 
               gtsam::Values& output_values);

private:

  double odom_threshold_;
  double pc_threshold_;

  int dim_; // dimension of value
  int r_dim_; // dimension of rotation part
  int t_dim_; // dimension of translation part

  gtsam::NonlinearFactorGraph nfg_odom_;
  gtsam::NonlinearFactorGraph nfg_special_;
  gtsam::NonlinearFactorGraph nfg_lc_;
  gtsam::NonlinearFactorGraph nfg_good_lc_; 
  gtsam::Matrix lc_adjacency_matrix_;
  gtsam::Matrix lc_distance_matrix_;
  graph_utils::Trajectory posesAndCovariances_odom_; 

  std::vector<char> special_symbols_;

  bool specialSymbol(char symb);

  void initializePrior(gtsam::PriorFactor<T> prior_factor);

  void updateOdom(gtsam::BetweenFactor<T> odom_factor, 
                  graph_utils::PoseWithCovariance &new_pose);

  bool isOdomConsistent(gtsam::BetweenFactor<T> lc_factor,
                        double& mahalanobis_dist);

  bool areLoopsConsistent(gtsam::BetweenFactor<T> lc_1, 
                          gtsam::BetweenFactor<T> lc_2,
                          double& mahalanobis_dist);

  void findInliers(gtsam::NonlinearFactorGraph &inliers);
};

#endif