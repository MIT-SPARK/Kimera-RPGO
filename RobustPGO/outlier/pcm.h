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

/* ------------------------------------------------------------------------ */
// Defines the behaviour of this backend.
enum class FactorType {
  UNCLASSIFIED = 0,
  ODOMETRY = 1, //
  FIRST_LANDMARK_OBSERVATION = 2,
  LOOP_CLOSURES = 3, // both between poses and landmark re-observations (may be more than 1)
  NONBETWEEN_FACTORS = 4, // not handled by PCM (may be more than 1)
};

// poseT can be gtsam::Pose2 or Pose3 for 3D vs 3D
// T can be PoseWithCovariance or PoseWithDistance based on
// If using Pcm or PcmDistance
template<class poseT, template <class> class T>
class Pcm : public OutlierRemoval{
public:
  Pcm(double threshold1, double threshold2,
      const std::vector<char>& special_symbols=std::vector<char>()):
        OutlierRemoval(),
        threshold1_(threshold1),  // TODO(Luca): threshold1 and 2 seem too generic as names
        threshold2_(threshold2),
        special_symbols_(special_symbols) {
    // check if templated value valid
    BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<poseT>));
  }
  ~Pcm() = default;
  // initialize with odometry detect threshold and pairwise consistency threshold

private:
  double threshold1_;
  double threshold2_;

  gtsam::NonlinearFactorGraph nfg_odom_; // NonlinearFactorGraph storing all odometry factors
  gtsam::NonlinearFactorGraph nfg_special_; // NonlinearFactorGraph storing all NonBetweenFactors
  gtsam::NonlinearFactorGraph nfg_lc_; // NonlinearFactorGraph storing all loop closure measurements
  gtsam::NonlinearFactorGraph nfg_good_lc_; // NonlinearFactorGraph storing the inliers found at last max clique query
  gtsam::Matrix lc_adjacency_matrix_; // adjacency matrix storing the consistency of each pair of loop closures
  gtsam::Matrix lc_distance_matrix_; // matrix storing distances between each pairs of loop closures

  Trajectory<poseT, T> trajectory_odom_; // trajectory storing keys and poses

  std::vector<char> special_symbols_; // these are the symbols corresponding to landmarks
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
    // we first classify the current factors into the following categories:
    FactorType type = FactorType::UNCLASSIFIED;
    // current logic: odometry and loop_closure are for those handled by outlier rej
    // mostly the betweenFactors and the PriorFactors
    // specials are those that are not handled: the rangefactors for example (uwb)

    // ==============================================================================
    // initialize trajectory for PCM if empty: requires either a single value or a prior factor
    if (trajectory_odom_.poses.size() == 0) {
      if (new_values.size() == 1 && new_factors.size() == 0) { // single value no prior case
        if (debug_) log<INFO>("Initializing without prior");
        initialize(new_values.keys()[0]);
        output_values.insert(new_values);
        return false; // nothing to optimize yet
      } else if (boost::dynamic_pointer_cast<gtsam::PriorFactor<poseT> >(new_factors[0])) { // prior factor case
        if (debug_) log<INFO>("Initializing with prior");
        gtsam::PriorFactor<poseT> prior_factor =
            *boost::dynamic_pointer_cast<gtsam::PriorFactor<poseT> >(new_factors[0]);
        initializeWithPrior(prior_factor);
        output_values.insert(new_values);
        output_nfg.add(new_factors); // assumption is that there is only one factor in new_factors
        return false; // noothing to optimize yet
      } else { // unknow case, fail
        log<WARNING> ("Unhandled initialization: first time PCM is called, it needs a particular input");
        return false;
      }
      if (debug_) log<INFO>("Initialized trajectory");
    }

    // ==============================================================================
    // now if the value size is one, should be an odometry // (could also have a loop closure if factor size > 1)
    if (new_factors.size() == 1 && new_factors[0]->keys().size() == 2 && new_values.size() == 1) {
      if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0])) {
        // specifically what outlier rejection handles
        gtsam::Symbol symb(new_values.keys()[0]);
        if (isSpecialSymbol(symb.chr())) { // is it a landmark?
          type = FactorType::FIRST_LANDMARK_OBSERVATION;
        }else{
          type = FactorType::ODOMETRY; // just regular odometry
        }
      } else {
        type = FactorType::NONBETWEEN_FACTORS; // unrecognized factor, not checked for outlier rejection
      }
    } else if (new_factors.size() > 0 && new_values.size() == 0) {
      type = FactorType::LOOP_CLOSURES; // both between poses and landmarks
    } else{
      // remains UNCLASSIFIED
    }

    // store new values:
    output_values.insert(new_values); // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)

    // ==============================================================================
    // handle differently depending on type
    bool doOptimize = false;
    switch (type) {
    case FactorType::ODOMETRY : // odometry, do not optimize
    {
      updateOdom(new_factors);
      doOptimize = false; // no need to optimize just for odometry
    } break;
    case FactorType::FIRST_LANDMARK_OBSERVATION : // landmark measurement, initialize
    {
      if (debug_) log<INFO>("New landmark observed");
      LandmarkMeasurements newMeasurement(new_factors);
      gtsam::Symbol symb(new_values.keys()[0]);
      landmarks_[symb] = newMeasurement;
      doOptimize = false; // no need to optimize just for odometry
    } break;
    case FactorType::LOOP_CLOSURES :
    {
      parseAndIncrementAdjMatrix(new_factors, output_values); // output values is just used to sanity check the keys
      findInliers(); // update inliers, // Find inliers with Pairwise consistent measurement set maximization
      doOptimize = true;
    } break;
    case FactorType::NONBETWEEN_FACTORS :
    {
      nfg_special_.add(new_factors);
      doOptimize = false;
    } break;
    default: // the remainders are specical loop closure cases, includes the "UNCLASSIFIED" case
    {
      nfg_special_.add(new_factors);
      if (new_factors.size() == 0) { // nothing added so no optimization
        doOptimize = false;
      }
      doOptimize = true;
    }
    }  // end switch

    output_nfg = buildGraphToOptimize();
    return doOptimize;
  } // end reject outliers

  /*! \brief save the PCM data
   *  saves the distance matrix (final) and also the clique size info
   *  - folder_path: path to directory to save results in
   */
  virtual void saveData(std::string folder_path) override{
    saveDistanceMatrix(folder_path);
    // saveCliqueSizeData(folder_path);
  }

protected:

  /*! \brief goes through the loop closures and updates the corresponding adjacency matrices,
   * in preparation for max clique
   */
  void parseAndIncrementAdjMatrix(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values output_values){
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

        if (!output_values.exists(nfg_factor.keys().front()) ||
            !output_values.exists(nfg_factor.keys().back())) {
          log<WARNING>("Cannot add loop closure with non-existing keys");
          continue;
        }

        // check if it is a landmark measurement loop closure
        gtsam::Symbol symbfrnt(nfg_factor.front());
        gtsam::Symbol symbback(nfg_factor.back());
        if (isSpecialSymbol(symbfrnt.chr()) || isSpecialSymbol(symbback.chr())) {
          // it is landmark loop closure
          gtsam::Key landmark_key = (isSpecialSymbol(symbfrnt.chr()) ?
              nfg_factor.front() : nfg_factor.back());

          if (debug_) log<INFO>("loop closing with landmark %1%") %
              gtsam::DefaultKeyFormatter(landmark_key);

          landmarks_[landmark_key].factors.add(nfg_factor);
          // grow adj matrix
          incrementLandmarkAdjMatrix(landmark_key);
        } else {
          // It is a proper loop closures
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
  }

  // check if a character is a special symbol as defined in constructor (typically these are the landmarks)
  bool isSpecialSymbol(char symb) const {
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

  /* ******************************************************************************* */
  // update the odometry: add new measurements to odometry trajectory tree
  void updateOdom(const gtsam::NonlinearFactorGraph& new_factors) {
    if (new_factors.size() != 1) {
      log<WARNING>("Factors passed to updateOdom should be of size one (the odom factor)");
    }
    gtsam::BetweenFactor<poseT> odom_factor =
        *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(new_factors[0]);
    nfg_odom_.add(odom_factor); // - store factor in nfg_odom_
    // update trajectory_odom_ (compose last value with new odom value)
    gtsam::Key new_key = odom_factor.keys().back();

    // construct pose with covariance for odometry measurement
    T<poseT> odom_delta(odom_factor);
    // Now get the latest pose in trajectory and compose
    gtsam::Key prev_key = odom_factor.keys().front();
    T<poseT> prev_pose;
    try {
      prev_pose =
          trajectory_odom_.poses[prev_key];
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
      // TODO: this in the future can be where we initialize key0! for future releases
    }

    // compose latest pose to odometry for new pose
    T<poseT> new_pose = prev_pose.compose(odom_delta);

    // add to trajectory
    trajectory_odom_.poses[new_key] = new_pose;
  }

  /* ******************************************************************************* */
  /*
   * odometry consistency check specialized to the PoseWithCovariance class
   */
  bool checkOdomConsistent(const PoseWithCovariance<poseT>& result, double& dist) const {
    dist = result.mahalanobis_norm(); // for PCM
    if (debug_) log<INFO>("odometry consistency distance: %1%") % dist;
    if (dist < threshold1_) {
      return true;
    }
    return false;
  }

  /* ******************************************************************************* */
  /*
   * odometry consistency check specialized to the PoseWithNode class
   */
  bool checkOdomConsistent(const PoseWithNode<poseT>& result, double& dist) {
    // For PcmSimple
    dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();

    if (debug_) log<INFO>("odometry consistency translation distance: %1%") % dist;
    if (debug_) log<INFO>("odometry consistency rotation distance: %1%") % rot_dist;
    if (dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* ******************************************************************************* */
  /*
   * general interface for odometry consistency check (both PCM and distance version)
   */
  bool isOdomConsistent(const gtsam::BetweenFactor<poseT>& lc_factor, double& dist) {
    // say: loop is between pose i and j
    gtsam::Key key_i = lc_factor.keys().front();     // extract the keys
    gtsam::Key key_j = lc_factor.keys().back();

    T<poseT> pij_odom, pji_lc, result;

    pij_odom = trajectory_odom_.getBetween(key_i, key_j);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc = T<poseT>(lc_factor).inverse();

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    return checkOdomConsistent(result, dist);
  }

  /* ******************************************************************************* */
  /*
   * pairwise loop consistency check specialized to the PoseWithCovariance class
   * TODO: delete this function, rename checkOdomConsistent to checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithCovariance<poseT>& result, double& dist) {
    dist = result.mahalanobis_norm();
    if (dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* ******************************************************************************* */
  /*
   * pairwise loop consistency check specialized to the PoseWithNode class
   * TODO: delete this function, rename checkOdomConsistent to checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithNode<poseT>& result, double& dist) {
    dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();
    if (dist < threshold1_ && rot_dist < threshold2_) {
      return true;
    }
    return false;
  }

  /* ******************************************************************************* */
  /*
   * general interface for loop consistency check (both PCM and distance version)
   * inputs are 2 loop closures (a,b) and (c,d), where a,b,c,d are keys
   */
  bool areLoopsConsistent(const gtsam::BetweenFactor<poseT>& a_lcBetween_b,
      const gtsam::BetweenFactor<poseT>& c_lcBetween_d,
      double& dist) {
    // check if two loop closures are consistent
    // say: loop closure 1 is (a,b)
    gtsam::Key key_a = a_lcBetween_b.keys().front();
    gtsam::Key key_b = a_lcBetween_b.keys().back();
    // say: loop closure 2 is (c,d)
    gtsam::Key key_c = c_lcBetween_d.keys().front();
    gtsam::Key key_d = c_lcBetween_d.keys().back();

    T<poseT> a_lc_b, c_lc_d;
    a_lc_b = T<poseT>(a_lcBetween_b);
    c_lc_d = T<poseT>(c_lcBetween_d);

    // find odometry from a to c
    T<poseT> a_odom_c = trajectory_odom_.getBetween(key_a, key_c);
    // find odometry from d to b
    T<poseT> b_odom_d = trajectory_odom_.getBetween(key_b, key_d);
    // check that d to b pose is consistent with pose from b to d
    T<poseT> a_path_d, d_path_b, loop;
    a_path_d = a_odom_c.compose(c_lc_d);
    d_path_b = a_path_d.inverse().compose(a_lc_b);
    loop = d_path_b.compose(b_odom_d);
    return checkLoopConsistent(loop, dist);
  }

  /* ******************************************************************************* */
  /*
   * augment adjacency matrix with an extra (pose-pose) loop closure
   */
  void incrementAdjMatrix() {
    // * pairwise consistency check (will also compare other loops - if loop fails we still store it, but not include in the optimization)
    // -- add 1 row and 1 column to lc_adjacency_matrix_;
    // -- populate extra row and column by testing pairwise consistency of new lc against all previous ones
    // -- compute max clique (done in the findInliers function)
    // -- add loops in max clique to a local variable nfg_good_lc (done in the updateOutputGraph function)
    // Using correspondence rowId (size_t, in adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
    size_t num_lc = nfg_lc_.size(); // number of loop closures so far, including the one we just added
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_adjacency_matrix_;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = lc_distance_matrix_;

      // now iterate through the previous loop closures and fill in last row + col of adjacency
      gtsam::BetweenFactor<poseT> factor_j = // latest loop closure: to be checked
                  *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(nfg_lc_[num_lc-1]);
      for (size_t i = 0; i < num_lc - 1; i++) { // compare it against all others
        gtsam::BetweenFactor<poseT> factor_i =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(nfg_lc_[i]);
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

  /* ******************************************************************************* */
  /*
   * augment adjacency matrix for a landmark loop closure
   */
  void incrementLandmarkAdjMatrix(const gtsam::Key& ldmk_key) {
    // pairwise consistency check for landmarks
    size_t num_lc = landmarks_[ldmk_key].factors.size(); // number measurements
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = landmarks_[ldmk_key].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = landmarks_[ldmk_key].dist_matrix;

      // now iterate through the previous loop closures and fill in last row + col of adjacency
      gtsam::BetweenFactor<poseT> factor_jl = // latest landmark loop closure: to be checked
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(
              landmarks_[ldmk_key].factors[num_lc-1]);

      // check it against all others
      for (size_t i = 0; i < num_lc - 1; i++) {
        gtsam::BetweenFactor<poseT> factor_il =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<poseT> >(
                landmarks_[ldmk_key].factors[i]);

        // check consistency
        gtsam::Key keyi = factor_il.keys().front();
        gtsam::Key keyj = factor_jl.keys().front();

        if (keyi == ldmk_key || keyj == ldmk_key) {
          log<WARNING>("Landmark observations should be connected pose -> landmark, discarding");
          return;
        }

        // factors are (i,l) and (j,l) and connect poses i,j to a landmark l
        T<poseT> i_pose_l, j_pose_l;
        i_pose_l = T<poseT>(factor_il);
        j_pose_l = T<poseT>(factor_jl);

        // find odometry from 1a to 2a
        T<poseT> i_odom_j = trajectory_odom_.getBetween(keyi, keyj);

        // check that lc_1 pose is consistent with pose from 1a to 1b
        T<poseT> i_path_l, loop;
        i_path_l = i_odom_j.compose(j_pose_l);
        loop = i_path_l.inverse().compose(i_pose_l);

        double dist;
        bool consistent = checkLoopConsistent(loop, dist);

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

  /* ******************************************************************************* */
  /*
   * Based on adjacency matrices, call maxclique to extract inliers
   */
  void findInliers() {
    if (debug_)
      log<INFO>("total loop closures registered: %1%") % nfg_lc_.size();

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

  /* ******************************************************************************* */
  /*
   * update the set of inliers to be outputted
   */
  gtsam::NonlinearFactorGraph buildGraphToOptimize() {
    gtsam::NonlinearFactorGraph output_nfg; // reset
    output_nfg.add(nfg_odom_);
    output_nfg.add(nfg_good_lc_); // computed by find inliers
    // add the good loop closures associated with landmarks
    std::unordered_map<gtsam::Key, LandmarkMeasurements>::iterator it = landmarks_.begin();
    while(it != landmarks_.end()) {
      output_nfg.add(it->second.consistent_factors);
      it++;
    }
    output_nfg.add(nfg_special_); // still need to update the class overall factorgraph
    return output_nfg;
  }

  /* ******************************************************************************* */
  /*
   * debug function: save max clique results
   */
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

  /* ******************************************************************************* */
  /*
   * debug function: save max clique results
   */
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
