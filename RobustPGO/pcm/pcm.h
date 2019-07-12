/*
Pairwise Consistency Maximization (PCM)
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
*/

#ifndef PCM_H
#define PCM_H

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR 

#include <fstream>
#include <sstream>
#include <math.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "RobustPGO/graph_utils/graph_utils.h" 
#include "RobustPGO/logger.h"
#include "RobustPGO/OutlierRemoval.h"

template<class T>
class PCM : public OutlierRemoval{
public:
  PCM(double odom_threshold, double pc_threshold, 
    std::vector<char> special_symbols=std::vector<char>()):
    odom_threshold_(odom_threshold), 
    pc_threshold_(pc_threshold),
    special_symbols_(special_symbols) {
  // check if templated value valid
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
}
  // initialize with odometry detect threshold and pairwise consistency threshold

private:
  double odom_threshold_;
  double pc_threshold_;

  gtsam::NonlinearFactorGraph nfg_odom_;
  gtsam::NonlinearFactorGraph nfg_special_;
  gtsam::NonlinearFactorGraph nfg_lc_;
  gtsam::NonlinearFactorGraph nfg_good_lc_; 
  gtsam::Matrix lc_adjacency_matrix_;
  gtsam::Matrix lc_distance_matrix_;
  graph_utils::Trajectory<T> posesAndCovariances_odom_; 

  std::vector<char> special_symbols_;

public:

  bool process(gtsam::NonlinearFactorGraph new_factors, 
               gtsam::Values new_values,
               gtsam::NonlinearFactorGraph& output_nfg, 
               gtsam::Values& output_values) {
    bool odometry = false;
    bool loop_closure = false;
    bool special_odometry = false;
    bool special_loop_closure = false;

    // current logic: odometry and loop_closure are for those handled by outlier rej
    // mostly the betweenFactors and the PriorFactors
    // specials are those that are not handled: the rangefactors for example (uwb)

    // initialize if pose is enoty: requrires either a single value or a prior factor
    log<INFO>("Pre init");
    if (posesAndCovariances_odom_.trajectory_poses.size() == 0) {
      // single value no prior case 
      if (new_values.size() == 1 && new_factors.size() == 0) {
        if (debug_) log<INFO>("Initializing without prior");
        initialize(new_values.keys()[0]);
        output_values.insert(new_values);
        return false; // nothing to optimize yet
      // prior factor case 
      } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<T> >(new_factors[0])) {
        if (debug_) log<INFO>("Initializing with prior");
        gtsam::PriorFactor<T> prior_factor =
            *boost::dynamic_pointer_cast<gtsam::PriorFactor<T> >(new_factors[0]);
        initializeWithPrior(prior_factor);
        output_values.insert(new_values);
        output_nfg.add(new_factors); // assumption is that there is only one factor in new_factors
        return false; // noothing to optimize yet 
      // unknow case, fail 
      } else {
        log<WARNING> ("Unhandled initialization case.");
        return false; 
      }
      if (debug_) log<INFO>("Initialized trajectory");
    }
    log<INFO>("No init");

    // now if the value size is one, should be an odometry
    // (could also have a loop closure if factor size > 1)
    if (new_values.size() == 1) {
      log<INFO>("odom candidate");
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[0]) ||
          (boost::dynamic_pointer_cast<gtsam::PriorFactor<T> >(new_factors[0]) &&
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[1]))) {
        // specifically what outlier rejection handles
        odometry = true; 
      } else {
        if (new_factors.size() < 2) {
          special_odometry = true;
        } else {
          special_loop_closure = true;
        }
      }

    } else if (new_factors.size() == 1 && new_values.size() == 0) {
      log<INFO>("loop closure candidate");
      // check if it is a between factor for classic loop closure case
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[0])) {
        loop_closure = true; 
      } else { // non-between factor (etc. range factor)
        special_loop_closure = true;
      }

    }

    // other cases will just be put through the special loop closures (which needs to be carefully considered)

    if (odometry) {
      // update posesAndCovariances_odom_;
      graph_utils::PoseWithCovariance<T> new_pose;

      // possible cases are that the first pose is a between factor or a prior 
      // also possible that there are two factors (a prior and a between)
      // (this triggers a loop closure)
      gtsam::NonlinearFactorGraph odom_factors, lc_factors; 

      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[0])) {
        // extract between factor 
        gtsam::BetweenFactor<T> odom_factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[0]);
        updateOdom(odom_factor, new_pose);
        odom_factors.add(odom_factor);
      } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<T> >(new_factors[0])) {
        // extract prior factor 
        gtsam::PriorFactor<T> prior_factor =
            *boost::dynamic_pointer_cast<gtsam::PriorFactor<T> >(new_factors[0]);
        updateOdom(prior_factor, new_pose);
        odom_factors.add(prior_factor);
        if (new_factors.size() == 2 && 
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[1])) {
          // a prior and a between
          lc_factors.add(new_factors[1]);
          loop_closure = true;
        }
      }

      // - store factor in nfg_odom_
      nfg_odom_.add(odom_factors);
      new_factors = lc_factors; // this will be carried over to the loop_closure section 

      if (!loop_closure) {
        // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)
        output_values.insert(new_values);
        output_nfg = gtsam::NonlinearFactorGraph(); // reset 
        output_nfg.add(nfg_odom_);
        output_nfg.add(nfg_good_lc_);
        output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
        return false; // no need to optimize just for odometry
      }
    } 

    if (loop_closure) { // in this case we should run consistency check to see if loop closure is good
      // * odometric consistency check (will only compare against odometry - if loop fails this, we can just drop it)
      // extract between factor 
      gtsam::BetweenFactor<T> nfg_factor =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(new_factors[0]);

      log<INFO>("Pre odom check");
      double odom_mah_dist; 
      if (isOdomConsistent(nfg_factor, odom_mah_dist)) {
        nfg_lc_.add(new_factors); // add factor to nfg_lc_

      } else {
        if (debug_) log<WARNING>("Discarded loop closure (inconsistent with odometry)");
        return false; // discontinue since loop closure not consistent with odometry 
      }
      
      log<INFO>("Pre find inliers");
      // Find inliers with Pairwise consistent measurement set maximization
      nfg_good_lc_ = gtsam::NonlinearFactorGraph(); // reset
      findInliers(nfg_good_lc_); // update nfg_good_lc_
      
      // * optimize and update values (for now just LM add others later)
      output_nfg = gtsam::NonlinearFactorGraph(); // reset
      output_nfg.add(nfg_odom_);
      output_nfg.add(nfg_good_lc_);
      output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
      return true; 

    } 

    if (debug_) log<INFO>("Adding odom or loop closure unhandled by outlier reject");
    if (special_odometry) {
      nfg_special_.add(new_factors);
      output_values.insert(new_values);
      // reset graph
      output_nfg = gtsam::NonlinearFactorGraph(); // reset
      output_nfg.add(nfg_odom_);
      output_nfg.add(nfg_good_lc_);
      output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
      return false;
    }

    // the remainders are speical loop closure cases
    nfg_special_.add(new_factors);
    output_values.insert(new_values);
    // reset graph
    output_nfg = gtsam::NonlinearFactorGraph(); // reset
    output_nfg.add(nfg_odom_);
    output_nfg.add(nfg_good_lc_);
    output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
    // nothing added  so no optimization
    if (new_factors.size() == 0) {
      return false; // nothing to optimize 
    }
    return true;
  }

  virtual bool processForcedLoopclosure(
      gtsam::NonlinearFactorGraph new_factors, 
      gtsam::Values new_values,
      gtsam::NonlinearFactorGraph& output_nfg, 
      gtsam::Values& output_values){
    // force loop closure (without outlier rejection)
    nfg_special_.add(new_factors);
    output_values.insert(new_values);
    // reset graph
    output_nfg = gtsam::NonlinearFactorGraph(); // reset
    output_nfg.add(nfg_odom_);
    output_nfg.add(nfg_good_lc_);
    output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
    return true;
  }

private:

  bool specialSymbol(char symb) {
    for (size_t i = 0; i < special_symbols_.size(); i++) {
      if (special_symbols_[i] == symb) return true;
    }
    return false; 
  }

  void initializeWithPrior(gtsam::PriorFactor<T> prior_factor) {
    T initial_value = prior_factor.prior();
    const int dim = graph_utils::getDim<T>();
    gtsam::Matrix covar = 
        Eigen::MatrixXd::Zero(dim, dim); // initialize as zero
    gtsam::Key initial_key = prior_factor.front();

    // construct initial pose with covar 
    graph_utils::PoseWithCovariance<T> initial_pose; 
    initial_pose.pose = initial_value;
    initial_pose.covariance_matrix = covar; 

    // populate posesAndCovariances_odom_
    posesAndCovariances_odom_.trajectory_poses[initial_key].pose = initial_pose;
    posesAndCovariances_odom_.start_id = initial_key;
    posesAndCovariances_odom_.end_id = initial_key;

    nfg_odom_.add(prior_factor); // add to initial odometry
  }

  void initialize(gtsam::Key initial_key) {
    const int dim = graph_utils::getDim<T>();
    gtsam::Matrix covar = 
        Eigen::MatrixXd::Zero(dim, dim); // initialize as zero

    // construct initial pose with covar 
    graph_utils::PoseWithCovariance<T> initial_pose; 
    initial_pose.pose = T();
    initial_pose.covariance_matrix = covar; 

    // populate posesAndCovariances_odom_
    posesAndCovariances_odom_.trajectory_poses[initial_key].pose = initial_pose;
    posesAndCovariances_odom_.start_id = initial_key;
    posesAndCovariances_odom_.end_id = initial_key;
  }

  void updateOdom(gtsam::BetweenFactor<T> odom_factor, 
                  graph_utils::PoseWithCovariance<T> &new_pose) {

    // update posesAndCovariances_odom_ (compose last value with new odom value)
    
    // first get measurement and covariance and key from factor
    T delta = odom_factor.measured(); 
    gtsam::Matrix covar =
        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>
        (odom_factor.get_noiseModel())->covariance(); // return covariance matrix

    gtsam::Key new_key = odom_factor.back();

    // construct pose with covariance for odometry measurement 
    graph_utils::PoseWithCovariance<T> odom_delta; 
    odom_delta.pose = delta; 
    odom_delta.covariance_matrix = covar; 

    // Now get the latest pose in trajectory and compose 
    gtsam::Key prev_key = odom_factor.front();
    graph_utils::PoseWithCovariance<T> prev_pose;
    try {
      prev_pose = 
        posesAndCovariances_odom_.trajectory_poses.at(prev_key).pose;
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
    }
    // compose latest pose to odometry for new pose
    new_pose = prev_pose.compose(odom_delta);
    // update trajectory 
    posesAndCovariances_odom_.end_id = new_key; // update end key 
    // add to trajectory 
    graph_utils::TrajectoryPose<T> new_trajectorypose; 
    new_trajectorypose.pose = new_pose;
    new_trajectorypose.id = new_key;
    posesAndCovariances_odom_.trajectory_poses[new_key] = new_trajectorypose; 
  }

  void updateOdom(gtsam::PriorFactor<T> prior_factor, 
                  graph_utils::PoseWithCovariance<T> &new_pose) {

    // update odometry when a prior added (considering multirobot use )
    gtsam::Matrix covar =
        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>
        (prior_factor.get_noiseModel())->covariance(); // return covariance matrix

    gtsam::Key new_key = prior_factor.key();

    // construct pose with covariance for new prior measurement 
    new_pose.covariance_matrix = covar;
    new_pose.pose = prior_factor.prior();
    // update trajectory 
    posesAndCovariances_odom_.end_id = new_key; // update end key 
    // add to trajectory 
    graph_utils::TrajectoryPose<T> new_trajectorypose; 
    new_trajectorypose.pose = new_pose;
    new_trajectorypose.id = new_key;
    posesAndCovariances_odom_.trajectory_poses[new_key] = new_trajectorypose; 
  }

  bool isOdomConsistent(gtsam::BetweenFactor<T> lc_factor,
                        double& mahalanobis_dist) {
    // assume loop is between pose i and j
    // extract the keys 
    gtsam::Key key_i = lc_factor.front();
    gtsam::Key key_j = lc_factor.back();
    
    graph_utils::PoseWithCovariance<T> pij_odom, pji_lc, result;

    // access (T_i,Cov_i) and (T_j, Cov_j) from trajectory_
    graph_utils::PoseWithCovariance<T> pi_odom, pj_odom; 
    pi_odom = posesAndCovariances_odom_.trajectory_poses[key_i].pose;
    pj_odom = posesAndCovariances_odom_.trajectory_poses[key_j].pose;

    pij_odom = pi_odom.between(pj_odom);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc.pose = lc_factor.measured().inverse(); 
    gtsam::Matrix pji_lc_covar = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>
        (lc_factor.get_noiseModel())->covariance();

    bool rotation_info = true;
    const int dim = graph_utils::getDim<T>();
    const int r_dim = graph_utils::getRotationDim<T>(); 
    const int t_dim = graph_utils::getTranslationDim<T>(); 
    if (std::isnan(pji_lc_covar.block(0,0,r_dim,r_dim).trace())) {
      rotation_info = false;
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(dim, dim);
      temp.block(r_dim,r_dim,t_dim,t_dim) = 
          pji_lc_covar.block(r_dim,r_dim,t_dim,t_dim);
      pji_lc_covar = temp; 
    }

    pji_lc.covariance_matrix = pji_lc_covar;

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    gtsam::Vector consistency_error = T::Logmap(result.pose);
    // check with threshold
    double threshold = odom_threshold_;
    // comput sqaure mahalanobis distance (the computation is wrong in robust mapper repo) 
    if (rotation_info) {
      mahalanobis_dist = std::sqrt(consistency_error.transpose() 
          * gtsam::inverse(result.covariance_matrix) * consistency_error);
    } else {
      mahalanobis_dist = std::sqrt(consistency_error.tail(t_dim).transpose()
          * gtsam::inverse(result.covariance_matrix.block(r_dim,r_dim,t_dim,t_dim))
          * consistency_error.tail(t_dim));
    }

    if (debug_) log<INFO>("odometry consistency distance: %1%") % mahalanobis_dist; 
    if (mahalanobis_dist < threshold) {
      return true;
    }
    
    return false;
  }

  bool areLoopsConsistent(gtsam::BetweenFactor<T> lc_1, 
                          gtsam::BetweenFactor<T> lc_2,
                          double& mahalanobis_dist) {
    // check if two loop closures are consistent 
    gtsam::Key key1a = lc_1.front();
    gtsam::Key key1b = lc_1.back();
    gtsam::Key key2a = lc_2.front();
    gtsam::Key key2b = lc_2.back();

    graph_utils::PoseWithCovariance<T> p1_lc_inv, p2_lc; 
    p1_lc_inv.pose = lc_1.measured().inverse();
    gtsam::Matrix p1_lc_covar = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>
        (lc_1.get_noiseModel())->covariance();

    bool rotation_info = true;
    const int dim = graph_utils::getDim<T>();
    const int r_dim = graph_utils::getRotationDim<T>(); 
    const int t_dim = graph_utils::getTranslationDim<T>(); 
    if (std::isnan(p1_lc_covar.block(0,0,r_dim,r_dim).trace())) {
      rotation_info = false; 
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(dim, dim);
      temp.block(r_dim,r_dim,t_dim,t_dim) = 
          p1_lc_covar.block(r_dim,r_dim,t_dim,t_dim);
      p1_lc_covar = temp;
    }
    p1_lc_inv.covariance_matrix = p1_lc_covar;

    p2_lc.pose = lc_2.measured();
    gtsam::Matrix p2_lc_covar = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>
        (lc_2.get_noiseModel())->covariance();

    if (std::isnan(p2_lc_covar.block(0,0,r_dim,r_dim).trace())) {
      rotation_info = false; 
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(dim, dim);
      temp.block(r_dim,r_dim,t_dim,t_dim) = 
          p2_lc_covar.block(r_dim,r_dim,t_dim,t_dim);
      p2_lc_covar = temp; 
    }
    p2_lc.covariance_matrix = p2_lc_covar;

    // find odometry from 1a to 2a 
    graph_utils::PoseWithCovariance<T> p1a_odom, p2a_odom, p1a2a_odom; 
    p1a_odom = posesAndCovariances_odom_.trajectory_poses[key1a].pose;
    p2a_odom = posesAndCovariances_odom_.trajectory_poses[key2a].pose;
    p1a2a_odom = p1a_odom.between(p2a_odom);

    // find odometry from 2b to 1b 
    graph_utils::PoseWithCovariance<T> p1b_odom, p2b_odom, p2b1b_odom; 
    p1b_odom = posesAndCovariances_odom_.trajectory_poses[key1b].pose;
    p2b_odom = posesAndCovariances_odom_.trajectory_poses[key2b].pose;
    p2b1b_odom = p2b_odom.between(p1b_odom);

    // check that lc_1 pose is consistent with pose from 1a to 1b 
    graph_utils::PoseWithCovariance<T> p1a2b, p1a1b, result; 
    p1a2b = p1a2a_odom.compose(p2_lc);
    p1a1b = p1a2b.compose(p2b1b_odom);
    result = p1a1b.compose(p1_lc_inv);

    gtsam::Vector consistency_error = T::Logmap(result.pose);

    // comput sqaure mahalanobis distance 
    if (rotation_info) {
      mahalanobis_dist = std::sqrt(consistency_error.transpose() 
          * gtsam::inverse(result.covariance_matrix) * consistency_error);
    } else {
      mahalanobis_dist = std::sqrt(consistency_error.tail(t_dim).transpose()
          * gtsam::inverse(result.covariance_matrix.block(r_dim,r_dim,t_dim,t_dim))
          * consistency_error.tail(t_dim));
    }

    // if (debug_) log<INFO>("loop consistency distance: %1%") % mahalanobis_dist; 
    if (mahalanobis_dist < pc_threshold_) {
      return true;
    }

    return false;
  }

  void findInliers(gtsam::NonlinearFactorGraph &inliers) {
    // * pairwise consistency check (will also compare other loops - if loop fails we still store it, but not include in the optimization)
    // -- add 1 row and 1 column to lc_adjacency_matrix_;
    // -- populate extra row and column by testing pairwise consistency of new lc against all previous ones
    // -- compute max clique
    // -- add loops in max clique to a local variable nfg_good_lc
    // NOTE: this will require a map from rowId (size_t, in adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
    size_t num_lc = nfg_lc_.size(); // number of loop closures so far
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized 
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_adjacency_matrix_; 
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_distance_matrix_;

      // now iterate through the previous loop closures and fill in last row + col 
      // of consistency matrix 
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<T> factor_i =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(nfg_lc_[i]);
        gtsam::BetweenFactor<T> factor_j =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(nfg_lc_[num_lc-1]);

        // check consistency 
        double mah_distance; 
        bool consistent = areLoopsConsistent(factor_i, factor_j, mah_distance);
        new_dst_matrix(num_lc-1, i) = mah_distance;
        new_dst_matrix(i, num_lc-1) = mah_distance;
        if (consistent) { 
          new_adj_matrix(num_lc-1, i) = 1; 
          new_adj_matrix(i, num_lc-1) = 1;
        }
      }
    }
    lc_adjacency_matrix_ = new_adj_matrix;
    lc_distance_matrix_ = new_dst_matrix;
    if (debug_) log<INFO>("total loop closures registered: %1%") % lc_adjacency_matrix_.rows();

    std::vector<int> max_clique_data;
    int max_clique_size = graph_utils::findMaxClique(lc_adjacency_matrix_, max_clique_data);
    if (debug_) log<INFO>("number of inliers: %1%") % max_clique_size;
    for (size_t i = 0; i < max_clique_size; i++) {
      // std::cout << max_clique_data[i] << " "; 
      inliers.add(nfg_lc_[max_clique_data[i]]);
    }
    std::ofstream file("log/pcm_dist_matrix.txt");
    if (file.is_open()) {
      file << lc_distance_matrix_;
    }
  }
};

#endif