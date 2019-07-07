// Authors: Pierre-Yves Lajoie, Yun Chang

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include "RobustPGO/max_clique_finder/findClique.h"
#include "RobustPGO/logger.h"

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
    // linear case so ignore all rotation components 
    Eigen::MatrixXd lin_cov_matrix = Eigen::MatrixXd::Zero(covariance_matrix.rows(),
        covariance_matrix.cols());
    lin_cov_matrix.block(3,3,3,3) = covariance_matrix.block(3,3,3,3);

    Eigen::MatrixXd lin_other_cov_matrix = Eigen::MatrixXd::Zero(other.covariance_matrix.rows(),
        other.covariance_matrix.cols());
    lin_other_cov_matrix.block(3,3,3,3) = other.covariance_matrix.block(3,3,3,3);

    out.covariance_matrix = Ha * lin_cov_matrix * Ha.transpose() +
        Hb * lin_other_cov_matrix * Hb.transpose();

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
    // linear case so ignore all rotation components 
    Eigen::MatrixXd lin_cov_matrix = Eigen::MatrixXd::Zero(covariance_matrix.rows(),
        covariance_matrix.cols());
    lin_cov_matrix.block(3,3,3,3) = covariance_matrix.block(3,3,3,3);

    Eigen::MatrixXd lin_other_cov_matrix = Eigen::MatrixXd::Zero(other.covariance_matrix.rows(),
        other.covariance_matrix.cols());
    lin_other_cov_matrix.block(3,3,3,3) = other.covariance_matrix.block(3,3,3,3);

    out.covariance_matrix = lin_other_cov_matrix - lin_cov_matrix;

    bool pos_semi_def = true;
    // compute the Cholesky decomp
    Eigen::LLT<Eigen::MatrixXd> lltCovar1(out.covariance_matrix.block(3,3,3,3));
    if(lltCovar1.info() == Eigen::NumericalIssue){  
      pos_semi_def = false;
    } 

    if (!pos_semi_def) { 
      other.pose.between(pose, Ha, Hb); // returns between in a frame 
      out.covariance_matrix = lin_cov_matrix - lin_other_cov_matrix;

      // Check if positive semidef 
      Eigen::LLT<Eigen::MatrixXd> lltCovar2(out.covariance_matrix);
      // if(lltCovar2.info() == Eigen::NumericalIssue){ 
      //   log<WARNING>("Warning: Covariance matrix between two poses not PSD"); 
      // } 
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