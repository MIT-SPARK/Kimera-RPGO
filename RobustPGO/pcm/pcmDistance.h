/*
Pairwise Consistency Maximization Simple (PcmDistance)
Differs from PCM in that it does not use covariances
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang
*/

#ifndef PCM_DIST_H
#define PCM_DIST_H

#include "RobustPGO/pcm/pcm.h"

namespace RobustPGO {

template<class T>
class PcmDistance : public Pcm<T>{
public:
  PcmDistance(double trans_threshold, double rot_threshold):
      OutlierRemoval(),
      trans_threshold_(trans_threshold), 
      rot_threshold_(rot_threshold) {
  // check if templated value valid
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
}

private:
  double trans_threshold_;
  double rot_threshold_;

  gtsam::Matrix lc_error_matrix_trans_;
  gtsam::Matrix lc_error_matrix_rot_;
  DistTrajectory<T> posesAndDistances_odom_;

private:

  void initializeWithPrior(gtsam::PriorFactor<T> prior_factor) {
    T initial_value = prior_factor.prior();
    gtsam::Key initial_key = prior_factor.front();

    // construct initial pose with covar 
    PoseWithDistance<T> initial_pose; 
    initial_pose.pose = initial_value;
    initial_pose.distance = 0; 

    // populate posesAndCovariances_odom_
    posesAndDistances_odom_.trajectory_poses[initial_key].pose = initial_pose;
    posesAndDistances_odom_.start_id = initial_key;
    posesAndDistances_odom_.end_id = initial_key;

    nfg_odom_.add(prior_factor); // add to initial odometry
  }

  void initialize(gtsam::Key initial_key) {
    // construct initial pose with covar 
    PoseWithDistance<T> initial_pose; 
    initial_pose.pose = T();
    initial_pose.distance = 0;

    // populate posesAndCovariances_odom_
    posesAndDistances_odom_.trajectory_poses[initial_key].pose = initial_pose;
    posesAndDistances_odom_.start_id = initial_key;
    posesAndDistances_odom_.end_id = initial_key;
  }

  void updateOdom(gtsam::BetweenFactor<T> odom_factor, 
                  PoseWithDistance<T> &new_pose) {
    // first get measurement and movement distance from factor
    T delta = odom_factor.measured(); 

    gtsam::Key new_key = odom_factor.back();

    double odom_dist = deta.translation().norm();

    // Now get the latest pose in trajectory and compose 
    gtsam::Key prev_key = odom_factor.front();
    PoseWithDistance<T> prev_pose;
    try {
      prev_pose = 
        posesAndDistances_odom_.trajectory_poses.at(prev_key).pose;
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
    }
    // compose latest pose to odometry for new pose
    new_pose.pose = prev_pose.pose.compose(delta);
    new_pose.distance = prev_pose.distance + odom_dist;
    // update trajectory 
    posesAndDistances_odom_.end_id = new_key; // update end key 
    // add to trajectory 
    posesAndDistances_odom_.trajectory_poses[new_key] = new_pose; 
  }

  bool isOdomConsistent(gtsam::BetweenFactor<T> lc_factor,
                        double& avg_rot_error, double& avg_trans_error) {
    // assume loop is between pose i and j
    // extract the keys 
    gtsam::Key key_i = lc_factor.front();
    gtsam::Key key_j = lc_factor.back();
    
    PoseWithDistance<T> pij_odom, pji_lc, result;

    // access (T_i,Cov_i) and (T_j, Cov_j) from trajectory_
    PoseWithDistance<T> pi_odom, pj_odom; 
    pi_odom = posesAndDistances_odom_.trajectory_poses[key_i].pose;
    pj_odom = posesAndDistances_odom_.trajectory_poses[key_j].pose;

    pij_odom.pose = pi_odom.pose.between(pj_odom.pose);
    pij_odom.distance = abs(pi_odom.distance - pj_odom.distance);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc.pose = lc_factor.measured().inverse();
    pji_lc.distance = lc_factor.measured().translation.norm();

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result.pose = pij_odom.pose.compose(pji_lc.pose);
    result.distance = pij_odom.distance + pji_lc.distance;

    gtsam::Vector consistency_error = T::Logmap(result.pose);

    const int r_dim = getRotationDim<T>(); 
    const int t_dim = getTranslationDim<T>();
    avg_trans_error = std::sqrt(consistency_error.tail(t_dim).transpose() *
        consistency_error.tail(t_dim)) / result.distance;
    avg_rot_error = std::sqrt(consistency_error.head(r_dim).transpose() *
        consistency_error.head(r_dim)) / result.distance;

    if (avg_trans_error < trans_threshold_ && avg_rot_error < rot_threshold_) {
      return true;
    }
    
    return false;
  }

  bool areLoopsConsistent(gtsam::BetweenFactor<T> lc_1, 
                          gtsam::BetweenFactor<T> lc_2,
                          double& avg_rot_error, double& avg_trans_error) {
    // check if two loop closures are consistent 
    gtsam::Key key1a = lc_1.front();
    gtsam::Key key1b = lc_1.back();
    gtsam::Key key2a = lc_2.front();
    gtsam::Key key2b = lc_2.back();

    PoseWithDistance<T> p1_lc_inv, p2_lc; 
    p1_lc_inv.pose = lc_1.measured().inverse();
    p1_lc_inv.distance = lc_1.measured().translation().norm();

    p2_lc.pose = lc_2.measured();
    p2_lc.distance = lc_2.measured().translation().norm();

    // find odometry from 1a to 2a 
    PoseWithDistance<T> p1a_odom, p2a_odom, p1a2a_odom; 
    p1a_odom = posesAndDistances_odom_.trajectory_poses[key1a].pose;
    p2a_odom = posesAndDistances_odom_.trajectory_poses[key2a].pose;
    p1a2a_odom.pose = p1a_odom.pose.between(p2a_odom.pose);
    p1a2a_odom.distance = abs(p1a_odom.distance - p2a_odom.distance);

    // find odometry from 2b to 1b 
    PoseWithDistance<T> p1b_odom, p2b_odom, p2b1b_odom; 
    p1b_odom = posesAndDistances_odom_.trajectory_poses[key1b].pose;
    p2b_odom = posesAndDistances_odom_.trajectory_poses[key2b].pose;
    p2b1b_odom.pose = p2b_odom.pose.between(p1b_odom.pose);
    p2b1b_odom.distance = abs(p2b_odom.distance - p1b_odom.distance);

    // check that lc_1 pose is consistent with pose from 1a to 1b 
    PoseWithDistance<T> p1a2b, p1a1b, result; 
    p1a2b.pose = p1a2a_odom.pose.compose(p2_lc.pose);
    p1a2b.distance = p1a2a_odom.distance + p2_lc.distance;
    p1a1b.pose = p1a2b.pose.compose(p2b1b_odom.pose);
    p1a1b.distance = p1a2b.distance + p2b1b_odom.distance;
    result.pose = p1a1b.pose.compose(p1_lc_inv.pose);
    result.distance = p1a1b.distance + p1_lc_inv.distance;

    gtsam::Vector consistency_error = T::Logmap(result.pose);

    const int r_dim = getRotationDim<T>(); 
    const int t_dim = getTranslationDim<T>();
    avg_trans_error = std::sqrt(consistency_error.tail(t_dim).transpose() *
        consistency_error.tail(t_dim)) / result.distance;
    avg_rot_error = std::sqrt(consistency_error.head(r_dim).transpose() *
        consistency_error.head(r_dim)) / result.distance;

    if (avg_trans_error < trans_threshold_ && avg_rot_error < rot_threshold_) {
      return true;
    }
    
    return false;
  }

  void incrementAdjMatrix() {
    // * pairwise consistency check (will also compare other loops - if loop fails we still store it, but not include in the optimization)
    // -- add 1 row and 1 column to lc_adjacency_matrix_;
    // -- populate extra row and column by testing pairwise consistency of new lc against all previous ones
    // -- compute max clique
    // -- add loops in max clique to a local variable nfg_good_lc
    // NOTE: this will require a map from rowId (size_t, in adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
    size_t num_lc = nfg_lc_.size(); // number of loop closures so far
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_error_matrix_rot = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_error_matrix_trans = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized 
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_adjacency_matrix_; 
      new_error_matrix_rot.topLeftCorner(num_lc - 1, num_lc - 1) = lc_error_matrix_rot_;
      new_error_matrix_trans.topLeftCorner(num_lc - 1, num_lc - 1) = lc_error_matrix_trans_;

      // now iterate through the previous loop closures and fill in last row + col 
      // of consistency matrix 
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<T> factor_i =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(nfg_lc_[i]);
        gtsam::BetweenFactor<T> factor_j =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<T> >(nfg_lc_[num_lc-1]);

        // check consistency 
        double rot_error, trans_error; 
        bool consistent = areLoopsConsistent(factor_i, factor_j, rot_error, trans_error);
        new_error_matrix_rot(num_lc-1, i) = rot_error;
        new_error_matrix_trans(num_lc-1, i) = trans_error;
        new_error_matrix_rot(i, num_lc-1) = rot_error;
        new_error_matrix_trans(i, num_lc-1) = trans_error;

        if (consistent) { 
          new_adj_matrix(num_lc-1, i) = 1; 
          new_adj_matrix(i, num_lc-1) = 1;
        }
      }
    }
    lc_adjacency_matrix_ = new_adj_matrix;
    lc_error_matrix_rot_ = new_error_matrix_rot;
    lc_error_matrix_trans_ = new_error_matrix_trans;
  }

  void findInliers(gtsam::NonlinearFactorGraph &inliers) {
    if (debug_) log<INFO>("total loop closures registered: %1%") % nfg_lc_.size();
    if (nfg_lc_.size() == 0) return;
    std::vector<int> max_clique_data;
    int max_clique_size = findMaxCliqueHeu(lc_adjacency_matrix_, max_clique_data);
    if (debug_) log<INFO>("number of inliers: %1%") % max_clique_size;
    for (size_t i = 0; i < max_clique_size; i++) {
      // std::cout << max_clique_data[i] << " "; 
      inliers.add(nfg_lc_[max_clique_data[i]]);
    }
  }

  void saveCliqueSizeData() {
    // TODO
    // log<INFO>("Saving clique size data");
    // std::stringstream filename;
    // filename << "log/clique_size" << std::setfill('0') 
    //     << std::setw(3) << lc_tr.rows() << ".txt";

    // std::ofstream cfile(filename.str());
    // if (cfile.is_open()) {
    //   // output for various thresholds 
    //   for (size_t i=0; i < lc_distance_matrix_.rows()-1; i++) {
    //     for (size_t j=i+1; j < lc_distance_matrix_.cols(); j++) {
    //       double threshold = lc_distance_matrix_(i,j); 
    //       Eigen::MatrixXd adj_matrix = (lc_distance_matrix_.array() < threshold).template cast<double>();
    //       std::vector<int> max_clique_data;
    //       int max_clique_size = findMaxClique(adj_matrix, max_clique_data);
    //       cfile << threshold << " " << max_clique_size << std::endl; 
    //     }
    //   }
    // }
  }

  void saveDistanceMatrix() {
    log<INFO>("Saving error matrices");
    std::ofstream file("log/PCM_rot_error_matrix.txt");
    if (file.is_open()) {
      file << lc_error_matrix_rot_;
    }
    std::ofstream file("log/PCM_trans_error_matrix.txt");
    if (file.is_open()) {
      file << lc_error_matrix_trans_;
    }
  }
};

typedef PcmDistance<gtsam::Pose2> PcmDistance2D;
typedef PcmDistance<gtsam::Pose3> PcmDistance3D;

}

#endif