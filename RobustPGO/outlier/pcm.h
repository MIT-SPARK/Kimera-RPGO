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
#include <iomanip>

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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

#include "RobustPGO/utils/geometry_utils.h"
#include "RobustPGO/utils/graph_utils.h"
#include "RobustPGO/logger.h"
#include "RobustPGO/outlier/OutlierRemoval.h"

namespace RobustPGO {

// poseT can be gtsam::Pose2 or Pose3 for 3D vs 3D
// T can be PoseWithCovariance or PoseWithDistance based on
// If using Pcm or PcmDistance

template<class poseT, template <class> class T>
class Pcm : public OutlierRemoval{
public:
  Pcm(double odom_threshold, double lc_threshold,
    const std::vector<char>& special_symbols=std::vector<char>()):
    OutlierRemoval(),
    odom_threshold_(odom_threshold),
    lc_threshold_(lc_threshold),
    special_symbols_(special_symbols) {
  // check if templated value valid
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<poseT>));
}
  ~Pcm() = default;
  // initialize with odometry detect threshold and pairwise consistency threshold

protected:
  double odom_threshold_;
  double lc_threshold_;

  gtsam::NonlinearFactorGraph nfg_odom_;
  gtsam::NonlinearFactorGraph nfg_special_;
  gtsam::NonlinearFactorGraph nfg_lc_;
  gtsam::NonlinearFactorGraph nfg_good_lc_;
  gtsam::Matrix lc_adjacency_matrix_;
  gtsam::Matrix lc_distance_matrix_;

  std::unordered_map< gtsam::Key, T<poseT> > trajectory_odom_;

  std::vector<char> special_symbols_;

public:

  virtual bool process(const gtsam::NonlinearFactorGraph& new_factors,
               const gtsam::Values& new_values,
               gtsam::NonlinearFactorGraph& output_nfg,
               gtsam::Values& output_values) override{
    bool odometry = false;
    bool loop_closures = false;
    bool special_odometry = false;

    // current logic: odometry and loop_closure are for those handled by outlier rej
    // mostly the betweenFactors and the PriorFactors
    // specials are those that are not handled: the rangefactors for example (uwb)

    // initialize if pose is enoty: requrires either a single value or a prior factor
    if (trajectory_odom_.size() == 0) {
      // single value no prior case
      if (new_values.size() == 1 && new_factors.size() == 0) {
        if (debug_) log<INFO>("Initializing without prior");
        initialize(new_values.keys()[0]);
        output_values.insert(new_values);
        return false; // nothing to optimize yet
      // prior factor case
      } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<poseT> >(new_factors[0])) {
        if (debug_) log<INFO>("Initializing with prior");
        gtsam::PriorFactor<poseT> prior_factor =
            *boost::dynamic_pointer_cast<gtsam::PriorFactor<poseT> >(new_factors[0]);
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

    // now if the value size is one, should be an odometry
    // (could also have a loop closure if factor size > 1)
    if (new_values.size() == 1) {
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0]) ||
          (boost::dynamic_pointer_cast<gtsam::PriorFactor<poseT> >(new_factors[0]) &&
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[1]))) {
        // specifically what outlier rejection handles
        odometry = true;
      } else {
        if (new_factors.size() < 2) {
          special_odometry = true;
        }
      }

    } else if (new_factors.size() > 0 && new_values.size() == 0) {
      loop_closures = true;
    }

    // other cases will just be put through the special loop closures (which needs to be carefully considered)

    if (odometry) {
      // update trajectory_odom_;
      T<poseT> new_pose;

      // possible cases are that the first pose is a between factor or a prior
      // also possible that there are two factors (a prior and a between)
      // (this triggers a loop closure)
      gtsam::NonlinearFactorGraph odom_factors, lc_factors;

      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0])) {
        // extract between factor
        gtsam::BetweenFactor<poseT> odom_factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0]);
        updateOdom(odom_factor, new_pose);
        odom_factors.add(odom_factor);
      }
      // - store factor in nfg_odom_
      nfg_odom_.add(odom_factors);

      if (!loop_closures) {
        // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)
        output_values.insert(new_values);
        output_nfg = gtsam::NonlinearFactorGraph(); // reset
        output_nfg.add(nfg_odom_);
        output_nfg.add(nfg_good_lc_);
        output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
        return false; // no need to optimize just for odometry
      }
    }

    if (loop_closures) {
      for (size_t i = 0; i < new_factors.size(); i++) {
        // iterate through the factors
        if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[i])) {
          // regular loop closure.
          // in this case we should run consistency check to see if loop closure is good
          // * odometric consistency check (will only compare against odometry
          // - if loop fails this, we can just drop it)
          // extract between factor
          gtsam::BetweenFactor<poseT> nfg_factor =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[i]);

          if (!output_values.exists(nfg_factor.front()) ||
              !output_values.exists(nfg_factor.back())) {
            log<WARNING>("Cannot add loop closure with non-existing keys");
            continue;
          }

          double odom_mah_dist;
          if (isOdomConsistent(nfg_factor, odom_mah_dist)) {
            nfg_lc_.add(new_factors[i]); // add factor to nfg_lc_

          } else {
            if (debug_) log<WARNING>("Discarded loop closure (inconsistent with odometry)");
            continue; // discontinue since loop closure not consistent with odometry
          }

          incrementAdjMatrix();

        } else {
          // add as special loop closure
          // the remainders are speical loop closure cases
          nfg_special_.add(new_factors[i]);
        }
      }
      // Find inliers with Pairwise consistent measurement set maximization
      nfg_good_lc_ = gtsam::NonlinearFactorGraph(); // reset
      findInliers(nfg_good_lc_); // update nfg_good_lc_

      output_nfg = gtsam::NonlinearFactorGraph(); // reset
      output_nfg.add(nfg_odom_);
      output_nfg.add(nfg_good_lc_);
      output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
      return true;
    }

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
    // nothing added  so no optimization
    if (new_factors.size() == 0) {
      return false; // nothing to optimize
    }

    // reset graph
    output_nfg = gtsam::NonlinearFactorGraph(); // reset
    output_nfg.add(nfg_odom_);
    output_nfg.add(nfg_good_lc_);
    output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
    return true;
  }

  virtual bool processForcedLoopclosure(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactorGraph& output_nfg,
      gtsam::Values& output_values) override{
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

  virtual void saveData(std::string folder_path) override{
    saveDistanceMatrix(folder_path);
    // saveCliqueSizeData(folder_path);
  }

protected:

  bool specialSymbol(char symb) {
    for (size_t i = 0; i < special_symbols_.size(); i++) {
      if (special_symbols_[i] == symb) return true;
    }
    return false;
  }

  void initializeWithPrior(const gtsam::PriorFactor<poseT>& prior_factor) {

    gtsam::Key initial_key = prior_factor.front();

    // construct initial pose with covar
    T<poseT> initial_pose(prior_factor);

    // populate trajectory_odom_
    trajectory_odom_[initial_key] = initial_pose;
    nfg_odom_.add(prior_factor); // add to initial odometry
  }

  void initialize(gtsam::Key initial_key) {

    T<poseT> initial_pose;

    // populate trajectory_odom_
    trajectory_odom_[initial_key] = initial_pose;
  }

  void updateOdom(const gtsam::BetweenFactor<poseT>& odom_factor,
                  T<poseT>& new_pose) {

    // update trajectory_odom_ (compose last value with new odom value)
    gtsam::Key new_key = odom_factor.back();

    // construct pose with covariance for odometry measurement
    T<poseT> odom_delta(odom_factor);

    // Now get the latest pose in trajectory and compose
    gtsam::Key prev_key = odom_factor.front();
    T<poseT> prev_pose;
    try {
      prev_pose =
        trajectory_odom_[prev_key];
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
    }

    // compose latest pose to odometry for new pose
    new_pose = prev_pose.compose(odom_delta);

    // add to trajectory
    trajectory_odom_[new_key] = new_pose;
  }

  bool isOdomConsistent(const gtsam::BetweenFactor<poseT>& lc_factor,
                        double& mahalanobis_dist) {
    // assume loop is between pose i and j
    // extract the keys
    gtsam::Key key_i = lc_factor.front();
    gtsam::Key key_j = lc_factor.back();

    T<poseT> pij_odom, pji_lc, result;

    // access (T_i,Cov_i) and (T_j, Cov_j) from trajectory_
    T<poseT> pi_odom, pj_odom;
    pi_odom = trajectory_odom_[key_i];
    pj_odom = trajectory_odom_[key_j];

    pij_odom = pi_odom.between(pj_odom);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc = T<poseT>(lc_factor).inverse();

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    mahalanobis_dist = result.norm();

    // if (debug_) log<INFO>("odometry consistency distance: %1%") % mahalanobis_dist;
    if (mahalanobis_dist < odom_threshold_) {
      return true;
    }

    return false;
  }

  bool areLoopsConsistent(const gtsam::BetweenFactor<poseT>& lc_1,
                          const gtsam::BetweenFactor<poseT>& lc_2,
                          double& mahalanobis_dist) {
    // check if two loop closures are consistent
    gtsam::Key key1a = lc_1.front();
    gtsam::Key key1b = lc_1.back();
    gtsam::Key key2a = lc_2.front();
    gtsam::Key key2b = lc_2.back();

    T<poseT> p1_lc_inv, p2_lc;
    p1_lc_inv = T<poseT>(lc_1).inverse();
    p2_lc = T<poseT>(lc_2);

    // find odometry from 1a to 2a
    T<poseT> p1a_odom, p2a_odom, p1a2a_odom;
    p1a_odom = trajectory_odom_[key1a];
    p2a_odom = trajectory_odom_[key2a];
    p1a2a_odom = p1a_odom.between(p2a_odom);

    // find odometry from 2b to 1b
    T<poseT> p1b_odom, p2b_odom, p2b1b_odom;
    p1b_odom = trajectory_odom_[key1b];
    p2b_odom = trajectory_odom_[key2b];
    p2b1b_odom = p2b_odom.between(p1b_odom);

    // check that lc_1 pose is consistent with pose from 1a to 1b
    T<poseT> p1a2b, p1a1b, result;
    p1a2b = p1a2a_odom.compose(p2_lc);
    p1a1b = p1a2b.compose(p2b1b_odom);
    result = p1a1b.compose(p1_lc_inv);

    mahalanobis_dist = result.norm();

    // if (debug_) log<INFO>("loop consistency distance: %1%") % mahalanobis_dist;
    if (mahalanobis_dist < lc_threshold_) {
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
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_adjacency_matrix_;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_distance_matrix_;

      // now iterate through the previous loop closures and fill in last row + col
      // of consistency matrix
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<poseT> factor_i =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(nfg_lc_[i]);
        gtsam::BetweenFactor<poseT> factor_j =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(nfg_lc_[num_lc-1]);

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
  }

  void findInliers(gtsam::NonlinearFactorGraph &inliers) {
    if (debug_) log<INFO>("total loop closures registered: %1%") % nfg_lc_.size();
    if (nfg_lc_.size() == 0) return;
    std::vector<int> max_clique_data;
    size_t max_clique_size = findMaxCliqueHeu(lc_adjacency_matrix_, max_clique_data);
    if (debug_) log<INFO>("number of inliers: %1%") % max_clique_size;
    for (size_t i = 0; i < max_clique_size; i++) {
      // std::cout << max_clique_data[i] << " ";
      inliers.add(nfg_lc_[max_clique_data[i]]);
    }
  }

  void saveCliqueSizeData(std::string folder_path) {
    log<INFO>("Saving clique size data");
    std::stringstream filename;
    filename << folder_path << "/clique_size" << std::setfill('0')
        << std::setw(3) << lc_distance_matrix_.rows() << ".txt";

    std::ofstream cfile(filename.str());
    if (cfile.is_open()) {
      // output for various thresholds
      size_t num_matrix_rows = lc_distance_matrix_.rows();
      size_t num_matrix_cols = lc_distance_matrix_.cols();
      for (size_t i=0; i < num_matrix_rows; i++) {
        for (size_t j=i+1; j < num_matrix_cols; j++) {
          double threshold = lc_distance_matrix_(i,j);
          Eigen::MatrixXd adj_matrix = (lc_distance_matrix_.array() < threshold).template cast<double>();
          std::vector<int> max_clique_data;
          int max_clique_size = findMaxCliqueHeu(adj_matrix, max_clique_data);
          cfile << threshold << " " << max_clique_data.size() << std::endl;
        }
      }
    }
  }

  void saveDistanceMatrix(std::string folder_path) {
    log<INFO>("Saving distance matrix");
    std::stringstream filename;
    filename << folder_path << "/dst_matrix" << std::setfill('0')
        << std::setw(3) << lc_distance_matrix_.rows() << ".txt";
    std::ofstream file(filename.str());
    if (file.is_open()) {
      file << lc_distance_matrix_;
    }
  }
};

typedef Pcm<gtsam::Pose2, PoseWithCovariance> Pcm2D;
typedef Pcm<gtsam::Pose3, PoseWithCovariance> Pcm3D;
typedef Pcm<gtsam::Pose2, PoseWithDistance> PcmDistance2D;
typedef Pcm<gtsam::Pose3, PoseWithDistance> PcmDistance3D;

}

#endif