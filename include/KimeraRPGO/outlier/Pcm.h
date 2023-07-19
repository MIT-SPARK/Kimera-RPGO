/*
Pairwise Consistency Maximization (PCM)
Backend solver class (Robust Pose Graph Optimizer)
author: Yun Chang, Luca Carlone
 */

#pragma once

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <math.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/outlier/OutlierRemoval.h"
#include "KimeraRPGO/utils/GeometryUtils.h"
#include "KimeraRPGO/utils/GraphUtils.h"

namespace KimeraRPGO {

/* ------------------------------------------------------------------------ */
// Defines the behaviour of this backend.
enum class FactorType {
  UNCLASSIFIED = 0,
  ODOMETRY = 1,  //
  FIRST_LANDMARK_OBSERVATION = 2,
  LOOP_CLOSURE = 3,
  // both between poses and landmark re-observations (may be more than 1)
  NONBETWEEN_FACTORS = 4,  // not handled by PCM (may be more than 1)
};

// poseT can be gtsam::Pose2 or Pose3 for 3D vs 3D
// T can be PoseWithCovariance or PoseWithDistance based on
// If using Pcm or PcmDistance
template <class poseT, template <class> class T>
class Pcm : public OutlierRemoval {
 public:
  Pcm(PcmParams params,
      MultiRobotAlignMethod align_method = MultiRobotAlignMethod::NONE,
      double align_gnc_probability = 0.99,
      const std::vector<char>& special_symbols = std::vector<char>())
      : OutlierRemoval(),
        params_(params),
        special_symbols_(special_symbols),
        total_lc_(0),
        total_good_lc_(0),
        multirobot_align_method_(align_method),
        multirobot_gnc_align_probability_(align_gnc_probability),
        odom_check_(true),
        loop_consistency_check_(true) {
    // check if templated value valid
    BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<poseT>));

    if (params_.odom_threshold < 0 || params_.odom_rot_threshold < 0 ||
        params_.odom_trans_threshold < 0) {
      odom_check_ = false;
    }

    if (params_.lc_threshold < 0 || params_.dist_rot_threshold < 0 ||
        params_.dist_trans_threshold < 0) {
      loop_consistency_check_ = false;
    }
  }
  ~Pcm() = default;
  // initialize with odometry detect threshold and pairwise consistency
  // threshold
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  PcmParams params_;

  // NonlinearFactorGraph storing all odometry factors
  gtsam::NonlinearFactorGraph nfg_odom_;

  // NonlinearFactorGraph storing all NonBetweenFactors
  gtsam::NonlinearFactorGraph nfg_special_;

  // storing loop closures and its adjacency matrix
  std::unordered_map<ObservationId, Measurements> loop_closures_;

  // trajectory maping robot prefix to its keys and poses
  std::unordered_map<char, Trajectory<poseT, T>> odom_trajectories_;

  // these are the symbols corresponding to landmarks
  std::vector<char> special_symbols_;

  // storing landmark measurements and its adjacency matrix
  std::unordered_map<gtsam::Key, Measurements> landmarks_;

  // store the vector of observations (loop closures)
  std::vector<ObservationId> loop_closures_in_order_;

  size_t total_lc_, total_good_lc_;

  // store the vector of ignored prefixes (loop closures to ignore)
  std::vector<char> ignored_prefixes_;

  // values and factors graphs for logging
  gtsam::NonlinearFactorGraph last_ouput_nfg_;
  gtsam::NonlinearFactorGraph odom_inconsistent_factors_;
  gtsam::NonlinearFactorGraph pairwise_inconsistent_factors_;

  // Multirobot initialization method
  MultiRobotAlignMethod multirobot_align_method_;
  double multirobot_gnc_align_probability_;
  // Keep track of the order of robots when applying world transforms
  std::vector<char> robot_order_;

  // Toggle odom and loop consistency check
  bool odom_check_;
  bool loop_consistency_check_;

 public:
  size_t getNumLC() { return total_lc_; }
  size_t getNumLCInliers() { return total_good_lc_; }
  size_t getNumOdomFactors() { return nfg_odom_.size(); }
  size_t getNumSpecialFactors() { return nfg_special_.size(); }

  /*! \brief Process new measurements and reject outliers
   *  process the new measurements and update the "good set" of measurements
   *  - new_factors: factors from the new measurements
   *  - new_values: linearization point of the new measurements
   *  - nfg: the factors after processing new measurements and outlier removal
   *  - values: the values after processing new measurements and outlier removal
   *  - returns: boolean of if optimization should be called or not
   */
  bool removeOutliers(const gtsam::NonlinearFactorGraph& new_factors,
                      const gtsam::Values& new_values,
                      gtsam::NonlinearFactorGraph* output_nfg,
                      gtsam::Values* output_values) override {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    // store new values:
    output_values->insert(new_values);
    if (new_factors.size() == 0) {
      // Done and nothing to optimize
      return false;
    }

    bool do_optimize = false;
    // ==============================================================================
    gtsam::NonlinearFactorGraph loop_closure_factors;
    for (size_t i = 0; i < new_factors.size(); i++) {
      if (NULL == new_factors[i]) continue;
      // we first classify the current factors into the following categories:
      FactorType type = FactorType::UNCLASSIFIED;
      // current logic: The factors handled by outlier rejection are the between
      // factors. Between factors can be classified broadly into odometry, loop
      // closures, and landmark observations

      // check if factor is a between factor
      if (factor_is_underlying_type<gtsam::BetweenFactor<poseT>>(
              new_factors[i])) {
        // specifically what outlier rejection handles
        gtsam::Key from_key = new_factors[i]->front();
        gtsam::Key to_key = new_factors[i]->back();
        gtsam::Symbol from_symb(from_key);
        gtsam::Symbol to_symb(to_key);
        if (isSpecialSymbol(from_symb.chr()) ||
            isSpecialSymbol(to_symb.chr())) {  // is it a landmark observation
          // if includes values, it is the first observation
          if (new_values.exists(from_key) || new_values.exists(to_key)) {
            type = FactorType::FIRST_LANDMARK_OBSERVATION;
          } else {
            // re-observation: counts as loop closure
            type = FactorType::LOOP_CLOSURE;
          }
        } else if (from_key + 1 == to_key && new_values.exists(to_key)) {
          // Note that here we assume we receive odometry incrementally
          type = FactorType::ODOMETRY;  // just regular odometry
        } else {
          type = FactorType::LOOP_CLOSURE;
        }
      } else {
        type = FactorType::NONBETWEEN_FACTORS;  // unrecognized factor, not
                                                // checked for outlier rejection
      }

      // ==============================================================================
      // handle differently depending on type
      switch (type) {
        case FactorType::ODOMETRY:  // odometry, do not optimize
        {
          updateOdom(new_factors[i], *output_values);
        } break;
        case FactorType::FIRST_LANDMARK_OBSERVATION:  // landmark measurement,
                                                      // initialize
        {
          if (debug_) log<INFO>("New landmark observed");
          Measurements newMeasurement;
          newMeasurement.factors.add(new_factors[i]);
          newMeasurement.consistent_factors.add(new_factors[i]);
          gtsam::Symbol symbfrnt(new_factors[i]->front());
          gtsam::Key landmark_key =
              (isSpecialSymbol(symbfrnt.chr()) ? new_factors[i]->front()
                                               : new_factors[i]->back());
          landmarks_[landmark_key] = newMeasurement;
          total_lc_++;
        } break;
        case FactorType::LOOP_CLOSURE: {
          if (new_factors[i]->front() != new_factors[i]->back()) {
            // add the the loop closure factors and process them together
            loop_closure_factors.add(new_factors[i]);
          } else {
            log<WARNING>("Attempting to close loop against self");
          }
        } break;
        case FactorType::NONBETWEEN_FACTORS: {
          nfg_special_.add(new_factors[i]);
          do_optimize = true;
        } break;
        default:  // the remainders are specical loop closure cases, includes
                  // the "UNCLASSIFIED" case
        {
          nfg_special_.add(new_factors[i]);
          do_optimize = true;
        }
      }  // end switch
    }
    auto max_clique_duration = std::chrono::milliseconds::zero();
    if (loop_closure_factors.size() > 0) {
      // update inliers
      std::unordered_map<ObservationId, size_t> num_new_loopclosures;
      parseAndIncrementAdjMatrix(
          loop_closure_factors, *output_values, &num_new_loopclosures);
      auto max_clique_start = std::chrono::high_resolution_clock::now();
      if (params_.incremental) {
        findInliersIncremental(num_new_loopclosures);
      } else {
        findInliers();
      }
      auto max_clique_end = std::chrono::high_resolution_clock::now();
      max_clique_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              max_clique_end - max_clique_start);
      // Find inliers with Pairwise consistent measurement set maximization
      do_optimize = true;
    }
    *output_nfg = buildGraphToOptimize();
    if (multirobot_align_method_ != MultiRobotAlignMethod::NONE &&
        robot_order_.size() > 1) {
      *output_values = multirobotValueInitialization(*output_values);
    }

    // End clock
    auto stop = std::chrono::high_resolution_clock::now();
    auto spin_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    if (debug_ && do_optimize)
      log<INFO>() << "PCM spin took " << spin_duration.count()
                  << " milliseconds. Detected " << total_lc_
                  << " total loop closures with " << total_good_lc_
                  << " inliers.";
    if (log_output_) {
      saveAdjacencyMatrix(log_folder_);
      logSpinStatus(
          spin_duration.count(), max_clique_duration.count(), log_folder_);
    }
    return do_optimize;
  }  // end reject outliers

  /*! \brief save the PCM data
   *  saves the distance matrix (final) and also the clique size info
   *  - folder_path: path to directory to save results in
   */
  void saveData(std::string folder_path) override {
    // TODO(Yun) save max clique results
    // TODO(Yun) Maybe just use log?
    // saveAdjacencyMatrix(folder_path);
    // saveCliqueSizeData(folder_path);
  }

  /*! \brief remove the last loop closure based on observation ID
   * and update the factors.
   * For example if Observation id is Obsid('a','c'), method
   * removes the last loop closure between robots a and c
   */
  EdgePtr removeLastLoopClosure(ObservationId id,
                                gtsam::NonlinearFactorGraph* updated_factors) {
    if (loop_closures_.find(id) == loop_closures_.end()) {
      return NULL;  // No loop closures in this container
    }
    // Update the measurements (delete last measurement)
    size_t numLC = loop_closures_[id].adj_matrix.rows();
    if (numLC <= 0) {
      return NULL;  // No more loop closures
    }
    size_t num_lc = loop_closures_[id].factors.size();
    Edge removed_edge = Edge(loop_closures_[id].factors[num_lc - 1]->front(),
                             loop_closures_[id].factors[num_lc - 1]->back());

    loop_closures_[id].factors.erase(
        std::prev(loop_closures_[id].factors.end()));
    if (loop_closures_[id].factors.size() < 2) {
      loop_closures_[id].consistent_factors = loop_closures_[id].factors;
    } else {
      // Update adjacent and distance matrix
      loop_closures_[id].adj_matrix =
          loop_closures_[id].adj_matrix.block(0, 0, numLC - 1, numLC - 1);
      loop_closures_[id].dist_matrix =
          loop_closures_[id].dist_matrix.block(0, 0, numLC - 1, numLC - 1);

      // Update the inliers
      std::vector<int> inliers_idx;
      size_t num_inliers =
          findMaxCliqueHeu(loop_closures_[id].adj_matrix, &inliers_idx);
      loop_closures_[id].consistent_factors =
          gtsam::NonlinearFactorGraph();  // reset
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        loop_closures_[id].consistent_factors.add(
            loop_closures_[id].factors[inliers_idx[i]]);
      }
    }

    *updated_factors = buildGraphToOptimize();
    return make_unique<Edge>(removed_edge);
  }

  /*! \brief remove the last loop closure regardless of observation ID
   * and update the factors.
   * Removes the last loop closure based on chronological order
   */
  EdgePtr removeLastLoopClosure(gtsam::NonlinearFactorGraph* updated_factors) {
    if (loop_closures_in_order_.size() == 0) return NULL;

    ObservationId last_obs = loop_closures_in_order_.back();
    loop_closures_in_order_.pop_back();
    return removeLastLoopClosure(last_obs, updated_factors);
  }

  /*! \brief Ignore all loop closures that involves certain prefix
   */
  void ignoreLoopClosureWithPrefix(
      char prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {
    if (std::find(ignored_prefixes_.begin(), ignored_prefixes_.end(), prefix) ==
        ignored_prefixes_.end())
      ignored_prefixes_.push_back(prefix);

    *updated_factors = buildGraphToOptimize();
  }

  /*! \brief Undo ignoreLoopClosureWithPrefix
   */
  void reviveLoopClosureWithPrefix(
      char prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {
    ignored_prefixes_.erase(
        std::remove(ignored_prefixes_.begin(), ignored_prefixes_.end(), prefix),
        ignored_prefixes_.end());

    *updated_factors = buildGraphToOptimize();
  }

  /*! \brief Get the vector of currently ignored prefixes
   */
  inline std::vector<char> getIgnoredPrefixes() { return ignored_prefixes_; }

  /*! \brief remove the prior factors of nodes that given prefix
   */
  void removePriorFactorsWithPrefix(
      const char& prefix,
      gtsam::NonlinearFactorGraph* updated_factors) {
    // First make copy of nfg_special_ where prior factors stored
    const gtsam::NonlinearFactorGraph nfg_special_copy = nfg_special_;
    // Clear nfg_special_
    nfg_special_ = gtsam::NonlinearFactorGraph();
    // Iterate and pick out non prior factors and prior factors without key with
    // prefix
    for (const auto& factor : nfg_special_copy) {
      const auto prior_factor =
          factor_pointer_cast<const gtsam::PriorFactor<poseT>>(factor);
      if (!prior_factor) {
        nfg_special_.add(factor);
      } else {
        gtsam::Symbol node(prior_factor->key());
        if (node.chr() != prefix) nfg_special_.add(factor);
      }
    }
    *updated_factors = buildGraphToOptimize();
    return;
  }

 protected:
  /*! \brief goes through the loop closures and updates the corresponding
   * adjacency matrices, in preparation for max clique
   */
  void parseAndIncrementAdjMatrix(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& output_values,
      std::unordered_map<ObservationId, size_t>* num_new_loopclosures) {
    for (size_t i = 0; i < new_factors.size(); i++) {
      // iterate through the factors
      // double check again that these are between factors
      auto factor_ptr =
          factor_pointer_cast<gtsam::BetweenFactor<poseT>>(new_factors[i]);
      if (factor_ptr) {
        const auto& nfg_factor = *factor_ptr;
        // regular loop closure.
        // in this case we should run consistency check to see if loop closure
        // is good
        // * odometric consistency check (will only compare against odometry
        // - if loop fails this, we can just drop it)
        // extract between factor
        if (!output_values.exists(nfg_factor.keys().front()) ||
            !output_values.exists(nfg_factor.keys().back())) {
          log<WARNING>("Cannot add loop closure with non-existing keys");
          continue;
        }

        // check if it is a landmark measurement loop closure
        gtsam::Symbol symbfrnt(nfg_factor.front());
        gtsam::Symbol symbback(nfg_factor.back());
        if (isSpecialSymbol(symbfrnt.chr()) ||
            isSpecialSymbol(symbback.chr())) {
          // it is landmark loop closure
          gtsam::Key landmark_key =
              (isSpecialSymbol(symbfrnt.chr()) ? nfg_factor.front()
                                               : nfg_factor.back());

          if (debug_) {
            log<INFO>() << "loop closing with landmark "
                        << gtsam::DefaultKeyFormatter(landmark_key);
          }

          landmarks_[landmark_key].factors.add(nfg_factor);
          total_lc_++;
          // grow adj matrix
          incrementLandmarkAdjMatrix(landmark_key);
        } else {
          // It is a proper loop closures
          double odom_dist;
          bool odom_consistent = false;
          if (symbfrnt.chr() == symbback.chr()) {
            odom_consistent = isOdomConsistent(nfg_factor, &odom_dist);
          } else {
            // odom consistency check only for intrarobot loop closures
            odom_consistent = true;
          }
          if (odom_consistent) {
            if (debug_)
              log<INFO>() << "loop closure between keys "
                          << gtsam::DefaultKeyFormatter(nfg_factor.front())
                          << " and "
                          << gtsam::DefaultKeyFormatter(nfg_factor.back());
            ObservationId obs_id(symbfrnt.chr(), symbback.chr());
            // detect which inter or intra robot loop closure this belongs to
            if (num_new_loopclosures->find(obs_id) ==
                num_new_loopclosures->end()) {
              num_new_loopclosures->insert(
                  std::pair<ObservationId, size_t>(obs_id, 1));
            } else {
              num_new_loopclosures->at(obs_id)++;
            }
            loop_closures_[obs_id].factors.add(nfg_factor);
            loop_closures_in_order_.push_back(obs_id);
            total_lc_++;
            if (loop_consistency_check_) {
              incrementAdjMatrix(obs_id, nfg_factor);
            }
          } else {
            if (debug_)
              log<WARNING>(
                  "Discarded loop closure (inconsistent with odometry)");
            continue;  // discontinue since loop closure not consistent with
                       // odometry
          }
        }

      } else {
        // add as special loop closure
        // the remainders are speical loop closure cases
        nfg_special_.add(new_factors[i]);
      }
    }
  }

  // check if a character is a special symbol as defined in constructor
  // (typically these are the landmarks)
  bool isSpecialSymbol(const char& symb) const {
    for (size_t i = 0; i < special_symbols_.size(); i++) {
      if (special_symbols_[i] == symb) return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  // update the odometry: add new measurements to odometry trajectory tree
  void updateOdom(const gtsam::NonlinearFactor::shared_ptr& new_factor,
                  const gtsam::Values& output_values) {
    // here we have values for reference checking and initialization if needed
    const auto& odom_factor =
        *factor_pointer_cast<const gtsam::BetweenFactor<poseT>>(new_factor);

    nfg_odom_.add(odom_factor);  // - store factor in nfg_odom_
    // update trajectory(compose last value with new odom value)
    gtsam::Key new_key = odom_factor.keys().back();
    gtsam::Key prev_key = odom_factor.keys().front();

    // extract prefix
    gtsam::Symbol sym = gtsam::Symbol(new_key);
    char prefix = sym.chr();

    // construct pose with covariance for odometry measurement
    T<poseT> odom_delta(odom_factor);

    if (odom_trajectories_.find(prefix) == odom_trajectories_.end()) {
      // prefix has not been seen before, add
      T<poseT> initial_pose;
      initial_pose.pose = output_values.at<poseT>(prev_key);
      // populate trajectories
      odom_trajectories_[prefix].poses[prev_key] = initial_pose;
      // add to robot order since seen for the first time
      robot_order_.push_back(prefix);
    }

    // Now get the latest pose in trajectory and compose
    T<poseT> prev_pose;
    try {
      prev_pose = odom_trajectories_[prefix].poses[prev_key];
    } catch (...) {
      log<WARNING>("Attempted to add odom to non-existing key. ");
    }

    // compose latest pose to odometry for new pose
    T<poseT> new_pose = prev_pose.compose(odom_delta);

    // add to trajectory
    odom_trajectories_[prefix].poses[new_key] = new_pose;
  }

  /* *******************************************************************************
   */
  /*
   * odometry consistency check specialized to the PoseWithCovariance class
   */
  bool checkOdomConsistent(const PoseWithCovariance<poseT>& result,
                           double* dist) const {
    *dist = result.mahalanobis_norm();  // for PCM
    if (debug_) {
      log<INFO>() << "odometry consistency distance: " << *dist;
    }

    if (*dist < params_.odom_threshold) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * odometry consistency check specialized to the PoseWithNode class
   */
  bool checkOdomConsistent(const PoseWithNode<poseT>& result, double* dist) {
    // For PcmSimple
    *dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();

    if (debug_) {
      log<INFO>() << "odometry consistency translation distance: " << *dist;
      log<INFO>() << "odometry consistency rotation distance: " << rot_dist;
    }
    if (*dist < params_.odom_trans_threshold &&
        rot_dist < params_.odom_rot_threshold) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * general interface for odometry consistency check (both PCM and distance
   * version)
   */
  bool isOdomConsistent(const gtsam::BetweenFactor<poseT>& lc_factor,
                        double* dist) {
    if (!odom_check_) return true;
    // say: loop is between pose i and j
    gtsam::Key key_i = lc_factor.keys().front();  // extract the keys
    gtsam::Key key_j = lc_factor.keys().back();
    gtsam::Symbol symb_i = gtsam::Symbol(key_i);
    gtsam::Symbol symb_j = gtsam::Symbol(key_j);

    T<poseT> pij_odom, pji_lc, result;

    if (symb_i.chr() != symb_j.chr()) {
      log<WARNING>(
          "Only check for odmetry consistency for intrarobot loop closures");
    }
    pij_odom = odom_trajectories_[symb_i.chr()].getBetween(key_i, key_j);

    // get pij_lc = (Tij_lc, Covij_lc) from factor
    pji_lc = T<poseT>(lc_factor).inverse();

    // check consistency (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc)
    result = pij_odom.compose(pji_lc);
    // if (debug_) result.pose.print("odom consistency check: ");

    return checkOdomConsistent(result, dist);
  }

  /* *******************************************************************************
   */
  /*
   * pairwise loop consistency check specialized to the PoseWithCovariance class
   * TODO: delete this function, rename checkOdomConsistent to
   * checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithCovariance<poseT>& result,
                           double* dist) {
    *dist = result.mahalanobis_norm();
    if (*dist < params_.lc_threshold) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * pairwise loop consistency check specialized to the PoseWithNode class
   * TODO: delete this function, rename checkOdomConsistent to
   * checkPairwiseConsistency and let it take a third argument (the threshold)
   */
  bool checkLoopConsistent(const PoseWithNode<poseT>& result, double* dist) {
    *dist = result.avg_trans_norm();
    double rot_dist = result.avg_rot_norm();
    if (*dist < params_.dist_trans_threshold &&
        rot_dist < params_.dist_rot_threshold) {
      return true;
    }
    return false;
  }

  /* *******************************************************************************
   */
  /*
   * general interface for loop consistency check (both PCM and distance
   * version) inputs are 2 loop closures (a,b) and (c,d), where a,b,c,d are keys
   */
  bool areLoopsConsistent(const gtsam::BetweenFactor<poseT>& a_lcBetween_b,
                          const gtsam::BetweenFactor<poseT>& c_lcBetween_d,
                          double* dist) {
    if (!loop_consistency_check_) return true;
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
    gtsam::Symbol symb_a = gtsam::Symbol(key_a);
    gtsam::Symbol symb_b = gtsam::Symbol(key_b);
    gtsam::Symbol symb_c = gtsam::Symbol(key_c);
    gtsam::Symbol symb_d = gtsam::Symbol(key_d);

    // make sure a and c has same prefix (from same robot)
    if (symb_a.chr() != symb_c.chr()) {
      // switch c and d if needed
      gtsam::Key temp = key_c;
      key_c = key_d;
      key_d = temp;
      symb_c = gtsam::Symbol(key_c);
      symb_d = gtsam::Symbol(key_d);
    }
    // find odometry from a to c
    if (symb_a.chr() != symb_c.chr()) {
      log<WARNING>("Attempting to get odometry between different trajectories");
    }
    T<poseT> a_odom_c =
        odom_trajectories_[symb_a.chr()].getBetween(key_a, key_c);
    // find odometry from d to b
    if (symb_b.chr() != symb_d.chr()) {
      log<WARNING>("Attempting to get odometry between different trajectories");
    }
    T<poseT> b_odom_d =
        odom_trajectories_[symb_b.chr()].getBetween(key_b, key_d);

    // check that d to b pose is consistent with pose from b to d
    T<poseT> a_path_d, d_path_b, loop;
    a_path_d = a_odom_c.compose(c_lc_d);
    d_path_b = a_path_d.inverse().compose(a_lc_b);
    loop = d_path_b.compose(b_odom_d);
    return checkLoopConsistent(loop, dist);
  }

  /* *******************************************************************************
   */
  /*
   * augment adjacency matrix with an extra (pose-pose) loop closure
   */
  void incrementAdjMatrix(const ObservationId& id,
                          const gtsam::BetweenFactor<poseT>& factor) {
    // * pairwise consistency check (will also compare other loops - if loop
    // fails we still store it, but not include in the optimization)
    // -- add 1 row and 1 column to lc_adjacency_matrix_;
    // -- populate extra row and column by testing pairwise consistency of new
    // lc against all previous ones
    // -- compute max clique (done in the findInliers function)
    // -- add loops in max clique to a local variable nfg_good_lc (done in the
    // updateOutputGraph function) Using correspondence rowId (size_t, in
    // adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
    size_t num_lc =
        loop_closures_[id].factors.size();  // number of loop closures so far,
                                            // including the one we just added
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          loop_closures_[id].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          loop_closures_[id].dist_matrix;

      // now iterate through the previous loop closures and fill in last row +
      // col of adjacency
      for (size_t i = 0; i < num_lc - 1;
           i++) {  // compare it against all others
        const auto& factor_i =
            *factor_pointer_cast<gtsam::BetweenFactor<poseT>>(
                loop_closures_[id].factors[i]);
        // check consistency
        double mah_distance = 0.0;
        bool consistent = areLoopsConsistent(factor_i, factor, &mah_distance);
        new_dst_matrix(num_lc - 1, i) = mah_distance;
        new_dst_matrix(i, num_lc - 1) = mah_distance;
        if (consistent) {
          new_adj_matrix(num_lc - 1, i) = 1;
          new_adj_matrix(i, num_lc - 1) = 1;
        }
      }
    }
    loop_closures_[id].adj_matrix = new_adj_matrix;
    loop_closures_[id].dist_matrix = new_dst_matrix;
  }

  /* *******************************************************************************
   */
  /*
   * augment adjacency matrix for a landmark loop closure
   */
  void incrementLandmarkAdjMatrix(const gtsam::Key& ldmk_key) {
    // pairwise consistency check for landmarks
    size_t num_lc = landmarks_[ldmk_key].factors.size();  // number measurements
    Eigen::MatrixXd new_adj_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    Eigen::MatrixXd new_dst_matrix = Eigen::MatrixXd::Zero(num_lc, num_lc);
    if (num_lc > 1) {
      // if = 1 then just initialized
      new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          landmarks_[ldmk_key].adj_matrix;
      new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
          landmarks_[ldmk_key].dist_matrix;

      // now iterate through the previous loop closures and fill in last row +
      // col of adjacency
      // latest landmark loop closure: to be checked
      const auto& factor_jl = *factor_pointer_cast<gtsam::BetweenFactor<poseT>>(
          landmarks_[ldmk_key].factors[num_lc - 1]);

      // check it against all others
      for (size_t i = 0; i < num_lc - 1; i++) {
        const auto& factor_il =
            *factor_pointer_cast<gtsam::BetweenFactor<poseT>>(
                landmarks_[ldmk_key].factors[i]);

        // check consistency
        gtsam::Key keyi = factor_il.keys().front();
        gtsam::Key keyj = factor_jl.keys().front();

        if (keyi == ldmk_key || keyj == ldmk_key) {
          log<WARNING>(
              "Landmark observations should be connected pose -> "
              "landmark, discarding");
          return;
        }

        // factors are (i,l) and (j,l) and connect poses i,j to a landmark l
        T<poseT> i_pose_l, j_pose_l;
        i_pose_l = T<poseT>(factor_il);
        j_pose_l = T<poseT>(factor_jl);

        gtsam::Symbol symb_i = gtsam::Symbol(keyi);
        gtsam::Symbol symb_j = gtsam::Symbol(keyj);

        // find odometry from 1a to 2a
        if (symb_i.chr() != symb_j.chr()) {
          log<WARNING>(
              "Attempting to get odometry between different trajectories");
        }
        T<poseT> i_odom_j =
            odom_trajectories_[symb_i.chr()].getBetween(keyi, keyj);

        // check that lc_1 pose is consistent with pose from 1a to 1b
        T<poseT> i_path_l, loop;
        i_path_l = i_odom_j.compose(j_pose_l);
        loop = i_path_l.inverse().compose(i_pose_l);

        double dist;
        bool consistent = checkLoopConsistent(loop, &dist);

        new_dst_matrix(num_lc - 1, i) = dist;
        new_dst_matrix(i, num_lc - 1) = dist;
        if (consistent) {
          new_adj_matrix(num_lc - 1, i) = 1;
          new_adj_matrix(i, num_lc - 1) = 1;
        }
      }
    }
    landmarks_[ldmk_key].adj_matrix = new_adj_matrix;
    landmarks_[ldmk_key].dist_matrix = new_dst_matrix;
  }

  /* *******************************************************************************
   */
  /*
   * Based on adjacency matrices, call maxclique to extract inliers
   */
  void findInliers() {
    if (debug_) {
      log<INFO>() << "total loop closures registered: " << total_lc_;
    }
    total_good_lc_ = 0;
    // iterate through loop closures and find inliers
    std::unordered_map<ObservationId, Measurements>::iterator it =
        loop_closures_.begin();
    while (it != loop_closures_.end()) {
      size_t num_inliers;
      if (loop_consistency_check_) {
        std::vector<int> inliers_idx;
        it->second.consistent_factors = gtsam::NonlinearFactorGraph();  // reset
        // find max clique
        num_inliers = findMaxCliqueHeu(it->second.adj_matrix, &inliers_idx);
        // update inliers, or consistent factors, according to max clique result
        for (size_t i = 0; i < num_inliers; i++) {
          it->second.consistent_factors.add(it->second.factors[inliers_idx[i]]);
        }
      } else {
        it->second.consistent_factors = it->second.factors;
        num_inliers = it->second.factors.size();
      }
      it++;
      total_good_lc_ = total_good_lc_ + num_inliers;
    }

    // iterate through landmarks and find inliers
    std::unordered_map<gtsam::Key, Measurements>::iterator it_ldmrk =
        landmarks_.begin();
    while (it_ldmrk != landmarks_.end()) {
      std::vector<int> inliers_idx;
      it_ldmrk->second.consistent_factors =
          gtsam::NonlinearFactorGraph();  // reset
      // find max clique
      size_t num_inliers =
          findMaxCliqueHeu(it_ldmrk->second.adj_matrix, &inliers_idx);
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it_ldmrk->second.consistent_factors.add(
            it_ldmrk->second.factors[inliers_idx[i]]);
      }
      it_ldmrk++;
      total_good_lc_ = total_good_lc_ + num_inliers;
    }
    if (debug_) {
      log<INFO>() << "number of inliers: " << total_good_lc_;
    }
  }

  /* *******************************************************************************
   */
  /*
   * TODO Incremental maxclique
   */
  void findInliersIncremental(
      const std::unordered_map<ObservationId, size_t>& num_new_loopclosures) {
    if (debug_) {
      log<INFO>() << "total loop closures registered: " << total_lc_;
    }
    total_good_lc_ = 0;
    // iterate through loop closures and find inliers
    std::unordered_map<ObservationId, size_t>::const_iterator new_lc_it =
        num_new_loopclosures.begin();
    while (new_lc_it != num_new_loopclosures.end()) {
      ObservationId robot_pair = new_lc_it->first;
      std::vector<int> inliers_idx;
      size_t prev_maxclique_size =
          loop_closures_[robot_pair].consistent_factors.size();
      // find max clique incrementally
      size_t num_inliers =
          findMaxCliqueHeuIncremental(loop_closures_[robot_pair].adj_matrix,
                                      new_lc_it->second,
                                      prev_maxclique_size,
                                      &inliers_idx);
      // update inliers, or consistent factors, according to max clique result
      // num_inliers will be zero if the previous inlier set should not be
      // changed
      if (num_inliers > 0) {
        loop_closures_[robot_pair].consistent_factors =
            gtsam::NonlinearFactorGraph();  // reset
        for (size_t i = 0; i < num_inliers; i++) {
          loop_closures_[robot_pair].consistent_factors.add(
              loop_closures_[robot_pair].factors[inliers_idx[i]]);
        }
      } else {
        // Set of inliers not modified. Don't reset consistent_factors
        num_inliers = prev_maxclique_size;
      }
      new_lc_it++;
    }

    // update total_good_lc_
    for (auto robot_pair_lc : loop_closures_) {
      total_good_lc_ =
          total_good_lc_ + robot_pair_lc.second.consistent_factors.size();
    }

    // iterate through landmarks and find inliers
    std::unordered_map<gtsam::Key, Measurements>::iterator it_ldmrk =
        landmarks_.begin();
    while (it_ldmrk != landmarks_.end()) {
      std::vector<int> inliers_idx;
      it_ldmrk->second.consistent_factors =
          gtsam::NonlinearFactorGraph();  // reset
      // find max clique
      size_t num_inliers =
          findMaxCliqueHeu(it_ldmrk->second.adj_matrix, &inliers_idx);
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it_ldmrk->second.consistent_factors.add(
            it_ldmrk->second.factors[inliers_idx[i]]);
      }
      it_ldmrk++;
      total_good_lc_ = total_good_lc_ + num_inliers;
    }
    if (debug_) {
      log<INFO>() << "number of inliers: " << total_good_lc_;
    }
  }

  /* *******************************************************************************
   */
  /*
   * update the set of inliers to be outputted
   */
  gtsam::NonlinearFactorGraph buildGraphToOptimize() {
    gtsam::NonlinearFactorGraph output_nfg;  // reset
    // important for gnc that we add the odom factors first
    output_nfg.add(nfg_odom_);  // add the odometry factors
    // important for gnc that we add the "special" non lc no odom factors second
    output_nfg.add(nfg_special_);
    // add the good loop closures
    std::unordered_map<ObservationId, Measurements>::iterator it =
        loop_closures_.begin();
    while (it != loop_closures_.end()) {
      if (std::find(ignored_prefixes_.begin(),
                    ignored_prefixes_.end(),
                    it->first.id1) == ignored_prefixes_.end() &&
          std::find(ignored_prefixes_.begin(),
                    ignored_prefixes_.end(),
                    it->first.id2) == ignored_prefixes_.end())
        output_nfg.add(it->second.consistent_factors);
      it++;
    }
    // add the good loop closures associated with landmarks
    std::unordered_map<gtsam::Key, Measurements>::iterator it_ldmrk =
        landmarks_.begin();
    while (it_ldmrk != landmarks_.end()) {
      output_nfg.add(it_ldmrk->second.consistent_factors);
      it_ldmrk++;
    }
    // still need to update the class overall factorgraph
    return output_nfg;
  }

  /*
   * use GNC pose averaging to estimate transforms between frames of the robots
   * and update the initial values
   */
  gtsam::Values multirobotValueInitialization(
      const gtsam::Values& input_values) {
    gtsam::Values initialized_values = input_values;
    if (robot_order_.size() == 0) {
      log<INFO>("No robot poses received. ");
      return initialized_values;
    }
    // Sort robot order from smallest prefix to larges
    std::sort(robot_order_.begin(), robot_order_.end());
    // Do not transform first robot
    initialized_values.update(getRobotOdomValues(robot_order_[0]));

    // Start estimating the frame-to-frame transforms between robots
    for (size_t i = 1; i < robot_order_.size(); i++) {
      // Get loop closures between robots robot_order_[0] and
      // robot_order_[i]
      const char& r0 = robot_order_[0];  // j = 0
      const char& ri = robot_order_[i];
      ObservationId obs_id(r0, ri);
      try {
        gtsam::NonlinearFactorGraph lc_factors =
            loop_closures_.at(obs_id).consistent_factors;
        // Create list of frame-to-fram transforms
        std::vector<poseT> T_w0_wi_measured;
        for (auto factor : lc_factors) {
          assert(factor != nullptr);
          const auto lc_ptr =
              factor_pointer_cast<gtsam::BetweenFactor<poseT>>(factor);
          assert(lc_ptr != nullptr);
          const auto& lc = *lc_ptr;

          gtsam::Symbol front = gtsam::Symbol(lc.key1());
          gtsam::Symbol back = gtsam::Symbol(lc.key2());
          poseT T_front_back = lc.measured();

          // Check order and switch if needed
          if (front.chr() != r0) {
            gtsam::Symbol front_temp = front;
            front = back;
            back = front_temp;
            T_front_back = T_front_back.inverse();
          }

          poseT T_w0_front, T_wi_back, T_w0_wi;
          // Get T_w1_fron and T_w2_back from stored trajectories
          T_w0_front = odom_trajectories_[r0].poses.at(front).pose;
          T_wi_back = odom_trajectories_[ri].poses.at(back).pose;

          T_w0_wi =
              T_w0_front.compose(T_front_back).compose(T_wi_back.inverse());
          T_w0_wi_measured.push_back(T_w0_wi);
        }
        // Pose averaging to find transform
        poseT T_w0_wi_est = gncRobustPoseAveraging(T_w0_wi_measured);
        initialized_values.update(
            getRobotOdomValues(robot_order_[i], T_w0_wi_est));
      } catch (const std::out_of_range& e) {
        log<WARNING>()
            << "No inter-robot loop closures between robots with prefix " << r0
            << " and " << ri << " for multirobot frame alignment.";
      }
    }
    return initialized_values;
  }

  /*
   * get the odometry poses of a robot as values, apply by some transform if
   * desired
   */
  gtsam::Values getRobotOdomValues(const char& robot_prefix,
                                   const poseT& transform = poseT()) {
    gtsam::Values robot_values;
    for (auto key_pose : odom_trajectories_.at(robot_prefix).poses) {
      poseT new_pose = transform.compose(key_pose.second.pose);
      robot_values.insert(key_pose.first, new_pose);
    }
    return robot_values;
  }

  /*
   * GNC Pose Averaging
   */
  poseT gncRobustPoseAveraging(const std::vector<poseT>& input_poses,
                               const double& rot_sigma = 0.1,
                               const double& trans_sigma = 0.5) {
    gtsam::Values initial;
    initial.insert(0, poseT());  // identity pose as initialization

    gtsam::NonlinearFactorGraph graph;
    size_t dim = getDim<poseT>();
    size_t r_dim = getRotationDim<poseT>();
    size_t t_dim = getTranslationDim<poseT>();
    gtsam::Vector sigmas;
    sigmas.resize(dim);
    sigmas.head(r_dim).setConstant(rot_sigma);
    sigmas.tail(t_dim).setConstant(trans_sigma);
    const gtsam::noiseModel::Diagonal::shared_ptr noise =
        gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    // add measurements
    for (auto pose : input_poses) {
      graph.add(gtsam::PriorFactor<poseT>(0, pose, noise));
    }

    gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;
    auto gnc =
        gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
            graph, initial, gncParams);

    if (multirobot_align_method_ == MultiRobotAlignMethod::L2) {
      gnc.setInlierCostThresholds(std::numeric_limits<double>::max());
    } else if (multirobot_align_method_ == MultiRobotAlignMethod::GNC) {
      gnc.setInlierCostThresholdsAtProbability(
          multirobot_gnc_align_probability_);
    } else {
      log<WARNING>(
          "Invalid multirobot alignment method in gncRobustPoseAveraging!");
    }

    gtsam::Values estimate = gnc.optimize();
    return estimate.at<poseT>(0);
  }

  /*
   * Save adjacency matrix to ObservationId_adj_matrix.txt
   */
  void saveAdjacencyMatrix(const std::string& folder_path) {
    for (auto measurement : loop_closures_) {
      ObservationId id = measurement.first;
      gtsam::Matrix adj_matrix = measurement.second.adj_matrix;

      // Save to file
      std::string filename =
          folder_path + "/" + id.id1 + "-" + id.id2 + "_adj_matrix.txt";
      std::ofstream outfile;
      outfile.open(filename);
      outfile << adj_matrix;
      outfile.close();
    }
  }

  /*
   * Log spin status (timing, number of loop closures, number of inliers)
   */
  void logSpinStatus(const int& spin_duration,
                     const int& find_inlier_duration,
                     const std::string& folder_path) {
    // Save to file
    std::string filename = folder_path + "/outlier_rejection_status.txt";
    std::ofstream outfile;
    outfile.open(filename, std::ofstream::out | std::ofstream::app);
    outfile << total_lc_ << " " << total_good_lc_ << " " << spin_duration << " "
            << find_inlier_duration << std::endl;
    outfile.close();
  }
};

typedef Pcm<gtsam::Pose2, PoseWithCovariance> Pcm2D;
typedef Pcm<gtsam::Pose3, PoseWithCovariance> Pcm3D;
typedef Pcm<gtsam::Pose2, PoseWithNode> PcmSimple2D;
typedef Pcm<gtsam::Pose3, PoseWithNode> PcmSimple3D;

}  // namespace KimeraRPGO
