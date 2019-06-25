// Authors: Pierre-Yves Lajoie, Yun Chang

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>

#include "RobustPGO/max_clique_finder/findClique.h"

#include <map>
#include <vector>

/** \namespace graph_utils
 *  \brief This namespace encapsulates utility functions to manipulate graphs
 */
namespace graph_utils {

/** \struct PoseWithCovariance
 *  \brief Structure to store a pose and its covariance data
 *  \currently supports gtsam::Pose2 and gtsam::Pose3
 */
template <class T>
struct PoseWithCovariance {
  /* variables ------------------------------------------------ */
  /* ---------------------------------------------------------- */
  T pose; // ex. gtsam::Pose3
  gtsam::Matrix covariance_matrix;

  /* method to combine two poses (along with their covariances) */
  /* ---------------------------------------------------------- */
  PoseWithCovariance compose(const PoseWithCovariance other) const {
    PoseWithCovariance<T> out; 
    gtsam::Matrix Ha, Hb;
    out.pose = pose.compose(other.pose, Ha, Hb);

    gtsam::Matrix tau1 = pose.AdjointMap();

    out.covariance_matrix = covariance_matrix +
        tau1 * other.covariance_matrix * tau1.transpose();

    return out;
  }
  
  /* method to invert a pose along with its covariance -------- */
  /* ---------------------------------------------------------- */
  PoseWithCovariance inverse() const {
    gtsam::Matrix Ha;
    PoseWithCovariance<T> out;
    out.pose = pose.inverse(Ha);
    out.covariance_matrix = Ha * covariance_matrix * Ha.transpose();

    return out;
  }

  /* method to find the transform between two poses ------------ */
  /* ----------------------------------------------------------- */
  PoseWithCovariance between(const PoseWithCovariance other) const {

    PoseWithCovariance<T> out; 
    gtsam::Matrix Ha, Hb;
    out.pose = pose.between(other.pose, Ha, Hb); // returns between in a frame 

    if (pose.equals(other.pose)) {
      out.covariance_matrix = 
        Eigen::MatrixXd::Zero(pose.dimension, pose.dimension);
      return out;
    }

    gtsam::Matrix tau1 = pose.AdjointMap();

    out.covariance_matrix = tau1.inverse() * 
        (other.covariance_matrix - covariance_matrix) * 
        tau1.transpose().inverse();

    bool pos_semi_def = true;
    // compute the Cholesky decomp
    Eigen::LLT<Eigen::MatrixXd> lltCovar1(out.covariance_matrix);
    if(lltCovar1.info() == Eigen::NumericalIssue){  
      pos_semi_def = false;
    } 

    if (!pos_semi_def) { 
      tau1 = other.pose.inverse().AdjointMap();
      out.covariance_matrix = tau1.inverse() * 
      (covariance_matrix - other.covariance_matrix) * 
      tau1.transpose().inverse();

      // Check if positive semidef 
      Eigen::LLT<Eigen::MatrixXd> lltCovar2(out.covariance_matrix);
      if(lltCovar2.info() == Eigen::NumericalIssue){ 
        log<WARNING>("Warning: Covariance matrix between two poses not PSD"); 
      } 
    }
    return out;
  }
};

/** \struct Transform
 *  \brief Structure defining a transformation between two poses
 */
template <class T>
struct Transform {
    gtsam::Key i, j;
    graph_utils::PoseWithCovariance<T> pose;
    bool is_separator;
};

/** \struct Transforms
 *  \brief Structure defining a std::map of transformations
 */
template <class T>
struct Transforms {
    gtsam::Key start_id, end_id;
    std::map<std::pair<gtsam::Key,gtsam::Key>, graph_utils::Transform<T>> transforms;
};

/** \struct TrajectoryPose
 *  \brief Structure defining a pose in a robot trajectory
 */
template <class T>
struct TrajectoryPose {
    gtsam::Key id;
    graph_utils::PoseWithCovariance<T> pose;
};

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 */
template <class T>
struct Trajectory {
    gtsam::Key start_id, end_id;
    std::map<gtsam::Key, graph_utils::TrajectoryPose<T>> trajectory_poses;
};

int findMaxClique(const Eigen::MatrixXd adjMatrix, std::vector<int>& max_clique) {
  // Compute maximum clique
  FMC::CGraphIO gio;
  gio.ReadEigenAdjacencyMatrix(adjMatrix, 0.0);
  int max_clique_size = 0;
  max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique);
  return max_clique_size;
}

template<class T>
static const size_t getRotationDim() {
  // get rotation dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object; 
  return sample_object.rotation().dimension;
}

template<class T>
static const size_t getTranslationDim() {
  // get translation dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object; 
  return sample_object.translation().dimension;
}

template<class T>
static const size_t getDim(){
  // get overall dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.dimension;
}

}
#endif