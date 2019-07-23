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
#include "RobustPGO/utils/type_utils.h"
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
    threshold1_(odom_threshold),
    threshold2_(lc_threshold),
    special_symbols_(special_symbols) {
  // check if templated value valid
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<poseT>));
}
  ~Pcm() = default;
  // initialize with odometry detect threshold and pairwise consistency threshold

private:
  double threshold1_;
  double threshold2_;

  gtsam::NonlinearFactorGraph nfg_odom_;
  gtsam::NonlinearFactorGraph nfg_special_;
  gtsam::NonlinearFactorGraph nfg_lc_;
  gtsam::NonlinearFactorGraph nfg_good_lc_;
  gtsam::Matrix lc_adjacency_matrix_;
  gtsam::Matrix lc_distance_matrix_;

  Trajectory<poseT, T> trajectory_odom_;

  std::vector<char> special_symbols_; // these should denote landmarks
  std::unordered_map< gtsam::Key, LandmarkMeasurements> landmarks_;

public:

  /*! \brief Process new measurements and reject outliers
   *  process the new measurements and update the "good set" of measurements
   *  - new_factors: factors from the new measurements
   *  - new_values: linearization point of the new measurements
   *  - nfg: the factors after processing new measurements and outlier removal
   *  - values: the values after processing new measurements and outlier removal
   *  - returns: boolean of if optimization should be called or not
   */
  virtual bool removeOutliers(const gtsam::NonlinearFactorGraph& new_factors,
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
    if (trajectory_odom_.poses.size() == 0) {
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
    if (new_values.size() == 1 && new_factors.size() == 1) {
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0])) {
        // specifically what outlier rejection handles
        odometry = true;
      } else {
        special_odometry = true;
      }

    } else if (new_factors.size() > 0 && new_values.size() == 0) {
      loop_closures = true;
    }

    // other cases will just be put through the special loop closures (which needs to be carefully considered)

    if (odometry) {
      // check if it is a landmark measurement
      gtsam::Symbol symb(new_values.keys()[0]);
      if (specialSymbol(symb.chr())) {
        // landmark measurement, initialize
        log<INFO>("New landmark observed");
        LandmarkMeasurements newMeasurement(new_factors);
        landmarks_[symb] = newMeasurement;
      } else {
        // update trajectory_odom_;
        // extract between factor
        gtsam::BetweenFactor<poseT> odom_factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0]);
        updateOdom(odom_factor);
        // - store factor in nfg_odom_
        nfg_odom_.add(odom_factor);
      }

      // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)
      output_values.insert(new_values);
      output_nfg = updateOutputGraph();

      return false; // no need to optimize just for odometry
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

          // check if it is a landmark measurement loop closure
          gtsam::Symbol symbfrnt(nfg_factor.front());
          gtsam::Symbol symbback(nfg_factor.back());
          if (specialSymbol(symbfrnt.chr()) || specialSymbol(symbback.chr())) {
            // it is landmark loop closure
            gtsam::Key landmark_key = (specialSymbol(symbfrnt.chr()) ?
                nfg_factor.front() : nfg_factor.back());

            log<INFO>("loop closing with landmark %1%") %
                gtsam::DefaultKeyFormatter(landmark_key);

            landmarks_[landmark_key].factors.add(nfg_factor);
            // grow adj matrix
            incrementLandmarkAdjMatrix(landmark_key);
          } else {

            double odom_dist;
            if (isOdomConsistent(nfg_factor, odom_dist)) {
              nfg_lc_.add(new_factors[i]); // add factor to nfg_lc_

            } else {
              if (debug_) log<WARNING>("Discarded loop closure (inconsistent with odometry)");
              continue; // discontinue since loop closure not consistent with odometry
            }

            incrementAdjMatrix();
          }

        } else {
          // add as special loop closure
          // the remainders are speical loop closure cases
          nfg_special_.add(new_factors[i]);
        }
      }
      // Find inliers with Pairwise consistent measurement set maximization
      findInliers(); // update inliers

      output_nfg = updateOutputGraph();
      return true;
    }

    if (special_odometry) {
      nfg_special_.add(new_factors);
      output_values.insert(new_values);
      // reset graph
      output_nfg = updateOutputGraph();
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
    output_nfg = updateOutputGraph();
    return true;
  }

  /*! \brief save the PCM data
   *  saves the distance matrix (final) and also the clique size info
   *  - folder_path: path to directory to save results in
   */
  virtual void saveData(std::string folder_path) override{
    saveDistanceMatrix(folder_path);
    // saveCliqueSizeData(folder_path);
  }

protected:

  // check if a character is a special symbol as defined in constructor
  bool specialSymbol(char symb) {
    for (size_t i = 0; i < special_symbols_.size(); i++) {
      if (special_symbols_[i] == symb) return true;
    }
    return false;
  }

  // initialize PCM with a prior factor
  void initializeWithPrior(const gtsam::PriorFactor<poseT>& prior_factor) {

    gtsam::Key initial_key = prior_factor.front();

    // construct initial pose with covar
    T<poseT> initial_pose(prior_factor);

    // populate trajectory_odom_
    trajectory_odom_.poses[initial_key] = initial_pose;
    nfg_odom_.add(prior_factor); // add to initial odometry
  }

  // initialize PCM without a prior factor
  void initialize(gtsam::Key initial_key) {

    T<poseT> initial_pose;

    // populate trajectory_odom_
    trajectory_odom_.poses[initial_key] = initial_pose;
  }

  // update the odometry: add new measurements to odometry trajectory tree
  void updateOdom(const gtsam::BetweenFactor<poseT>& odom_factor) {

    // update trajectory_odom_ (compose last value with new odom value)
    gtsam::Key new_key = odom_factor.back();

    // construct pose with covariance for odometry measurement
    T<poseT> odom_delta(odom_factor);

    // Now get the latest pose in trajectory and compose
    gtsam::Key prev_key = odom_factor.front();
    T<poseT> prev_pose;
    try {
      prev_pose =
        trajectory_odom_.poses[prev_key];
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
    }

    // compose latest pose to odometry for new pose
    T<poseT> new_pose = prev_pose.compose(odom_delta);

    // add to trajectory
    trajectory_odom_.poses[new_key] = new_pose;
  }

  bool checkOdomConsistent(const PoseWithCovariance<poseT>& result,
        double& dist) {
    // For Pcm
    dist = result.mahalanobis_norm();
    if (debug_) log<INFO>("odometry consistency distance: %1%") % dist;
    if (dist < threshold1_) {
      return true;
    }
    return false;
  }

  bool checkOdomConsistent(const PoseWithNode<poseT>& result,
        double& dist) {
    // For PcmSimple
    dist = result.trans_norm();
    double rot_dist = result.rot_norm();

    if (debug_) log<INFO>("odometry consistency translation distance: %1%") % dist;
    if (debug_) log<INFO>("odometry consistency rotation distance: %1%") % rot_dist;
    if (dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  // check if loop closure is consistent with the odometry easurements
  bool isOdomConsistent(const gtsam::BetweenFactor<poseT>& lc_factor,
                        double& dist) {
    // assume loop is between pose i and j
    // extract the keys
    gtsam::Key key_i = lc_factor.front();
    gtsam::Key key_j = lc_factor.back();

    T<poseT> pij_odom, pji_lc, result;

    pij_odom = trajectory_odom_.getBetween(key_i, key_j);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc = T<poseT>(lc_factor).inverse();

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    return checkOdomConsistent(result, dist);
  }

  bool checkLoopConsistent(const PoseWithCovariance<poseT>& result,
        double& dist) {
    // For Pcm
    dist = result.mahalanobis_norm();
    if (dist < threshold2_) {
      return true;
    }
    return false;
  }

  bool checkLoopConsistent(const PoseWithNode<poseT>& result,
        double& dist) {
    // For PcmSimple
    dist = result.trans_norm();
    double rot_dist = result.rot_norm();
    if (dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  // Main PCM function: Check if a pair of loop closures is consistent in measurement
  bool areLoopsConsistent(const gtsam::BetweenFactor<poseT>& lc_1,
                          const gtsam::BetweenFactor<poseT>& lc_2,
                          double& dist) {
    // check if two loop closures are consistent
    gtsam::Key key1a = lc_1.front();
    gtsam::Key key1b = lc_1.back();
    gtsam::Key key2a = lc_2.front();
    gtsam::Key key2b = lc_2.back();

    T<poseT> p1_lc_inv, p2_lc;
    p1_lc_inv = T<poseT>(lc_1).inverse();
    p2_lc = T<poseT>(lc_2);

    // find odometry from 1a to 2a
    T<poseT> p1a2a_odom = trajectory_odom_.getBetween(key1a, key2a);

    // find odometry from 2b to 1b
    T<poseT> p2b1b_odom = trajectory_odom_.getBetween(key2b, key1b);

    // check that lc_1 pose is consistent with pose from 1a to 1b
    T<poseT> p1a2b, p1a1b, result;
    p1a2b = p1a2a_odom.compose(p2_lc);
    p1a1b = p1a2b.compose(p2b1b_odom);
    result = p1a1b.compose(p1_lc_inv);

    return checkLoopConsistent(result, dist);
  }

  // increment the adjacency matrix for the main loop closures
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

  // increment adjacency matrix for a landmark
  void incrementLandmarkAdjMatrix(const gtsam::Key& ldmk_key) {
    // pairwise consistency check for landmarks
    size_t num_lc = landmarks_[ldmk_key].factors.size(); // number measurements
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = landmarks_[ldmk_key].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = landmarks_[ldmk_key].dist_matrix;

      // now iterate through the previous loop closures and fill in last row + col
      // of consistency matrix
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<poseT> factor_i =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(
              landmarks_[ldmk_key].factors[i]);
        gtsam::BetweenFactor<poseT> factor_j =
              *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(
              landmarks_[ldmk_key].factors[num_lc-1]);

        // check consistency
        gtsam::Key keyi = factor_i.front();
        gtsam::Key keyj = factor_j.front();

        if (keyi == ldmk_key || keyj == ldmk_key) {
          log<WARNING>("Landmark observations should be connected pose -> landmark, discarding");
          return;
        }

        T<poseT> pil_inv, pjl;
        pil_inv = T<poseT>(factor_i).inverse();
        pjl = T<poseT>(factor_j);

        // find odometry from 1a to 2a
        T<poseT> pij_odom = trajectory_odom_.getBetween(keyi, keyj);

        // check that lc_1 pose is consistent with pose from 1a to 1b
        T<poseT> pil, result;
        pil = pij_odom.compose(pjl);
        result = pil.compose(pil_inv);

        double dist;
        bool consistent = checkLoopConsistent(result, dist);

        new_dst_matrix(num_lc-1, i) = dist;
        new_dst_matrix(i, num_lc-1) = dist;
        if (consistent) {
          new_adj_matrix(num_lc-1, i) = 1;
          new_adj_matrix(i, num_lc-1) = 1;
        }
      }
    }
    landmarks_[ldmk_key].adj_matrix = new_adj_matrix;
    landmarks_[ldmk_key].dist_matrix = new_dst_matrix;
  }

  // Based on adjacency matrices, call maxclique to extract inliers
  void findInliers() {
    if (debug_) log<INFO>("total loop closures registered: %1%") % nfg_lc_.size();
    if (nfg_lc_.size() != 0) {
      nfg_good_lc_ = gtsam::NonlinearFactorGraph(); // reset
      std::vector<int> max_clique_data;
      size_t max_clique_size = findMaxCliqueHeu(lc_adjacency_matrix_, max_clique_data);
      if (debug_) log<INFO>("number of inliers: %1%") % max_clique_size;
      for (size_t i = 0; i < max_clique_size; i++) {
        // std::cout << max_clique_data[i] << " ";
        nfg_good_lc_.add(nfg_lc_[max_clique_data[i]]);
      }
    }

    // iterate through landmarks and find inliers
    std::unordered_map<gtsam::Key, LandmarkMeasurements>::iterator it = landmarks_.begin();
    while(it != landmarks_.end()) {
      std::vector<int> inliers_idx;
      it->second.consistent_factors = gtsam::NonlinearFactorGraph(); // reset
      // find max clique
      size_t num_inliers = findMaxCliqueHeu(it->second.adj_matrix, inliers_idx);
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it->second.consistent_factors.add(it->second.factors[inliers_idx[i]]);
      }
      it++;
    }
  }

  // update the set of inliers to be outputted
  gtsam::NonlinearFactorGraph updateOutputGraph() {
    gtsam::NonlinearFactorGraph output_nfg; // reset
    output_nfg.add(nfg_odom_);
    output_nfg.add(nfg_good_lc_);
    // add the good loop closures associated with landmarks
    std::unordered_map<gtsam::Key, LandmarkMeasurements>::iterator it = landmarks_.begin();
    while(it != landmarks_.end()) {
      output_nfg.add(it->second.consistent_factors);
      it++;
    }
    output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
    return output_nfg;
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
typedef Pcm<gtsam::Pose2, PoseWithNode> PcmSimple2D;
typedef Pcm<gtsam::Pose3, PoseWithNode> PcmSimple3D;

}

#endif