#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>
#include <map>
#include <vector>

#include "kimera_rpgo/utils/utils.h"
namespace kimera_rpgo {

/** \getting the dimensions of various Lie types
 *   \simple helper functions */
template <class T>
static size_t getRotationDim() {
  // get rotation dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.rotation().dimension;
}

template <class T>
static size_t getTranslationDim() {
  // get translation dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.translation().size();
}

template <class T>
static size_t getDim() {
  // get overall dimension of some gtsam object
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  T sample_object;
  return sample_object.dimension;
}

struct Pose {
  using Ptr = std::unique_ptr<Pose>;
  virtual ~Pose() = default;
  virtual Pose::Ptr compose(const Pose::Ptr& other) const = 0;

  virtual Pose::Ptr inverse() const = 0;

  virtual Pose::Ptr between(const Pose::Ptr& other) const = 0;
};

/** \struct PoseWithCovariance
 *  \brief Structure to store a pose and its covariance data
 *  \currently supports gtsam::Pose2 and gtsam::Pose3
 */
template <class T>
struct PoseWithCovariance : Pose {
  using Ptr = std::unique_ptr<PoseWithCovariance>;
  /* variables ------------------------------------------------ */
  /* ---------------------------------------------------------- */
  T pose;  // ex. gtsam::Pose3
  gtsam::Matrix covariance_matrix;
  bool rotation_info = true;

  /* default constructor -------------------------------------- */
  PoseWithCovariance() {
    pose = T();
    const size_t dim = getDim<T>();
    gtsam::Matrix covar = Eigen::MatrixXd::Zero(dim, dim);  // initialize as zero
    covariance_matrix = covar;
  }

  /* basic constructor ---------------------------------------- */
  PoseWithCovariance(T pose_in, gtsam::Matrix matrix_in) {
    pose = pose_in;
    covariance_matrix = matrix_in;
  }

  /* construct from gtsam prior factor ------------------------ */
  explicit PoseWithCovariance(const gtsam::PriorFactor<T>& prior_factor) {
    T value = prior_factor.prior();
    const size_t dim = getDim<T>();
    gtsam::Matrix covar = Eigen::MatrixXd::Zero(dim, dim);  // initialize as zero

    pose = value;
    covariance_matrix = covar;
  }

  /* construct from gtsam between factor  --------------------- */
  explicit PoseWithCovariance(const gtsam::BetweenFactor<T>& between_factor) {
    pose = between_factor.measured();
    gtsam::Matrix covar =
        factor_pointer_cast<gtsam::noiseModel::Gaussian>(between_factor.noiseModel())
            ->covariance();

    // prevent propagation of nan values in the edge case
    const int dim = getDim<T>();
    const int r_dim = getRotationDim<T>();
    const int t_dim = getTranslationDim<T>();
    rotation_info = true;
    if (std::isnan(covar.block(0, 0, r_dim, r_dim).trace())) {
      rotation_info = false;
      // only keep translation part
      Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(
          dim, dim);  // TODO(Yun): I wonder if this can cause issues: ...
      // ... later you invert this matrix, which now contains a bunch of zero
      // (it is not full rank)
      temp.block(r_dim, r_dim, t_dim, t_dim) = covar.block(r_dim, r_dim, t_dim, t_dim);
      covar = temp;
    }
    covariance_matrix = covar;
  }

  template <typename pointT>
  explicit PoseWithCovariance(const gtsam::PoseToPointFactor<T, pointT>& factor) {
    const int dim = getDim<T>();
    const int r_dim = getRotationDim<T>();
    const int t_dim = getTranslationDim<T>();
    T temp;
    gtsam::Matrix pose_matrix = Eigen::MatrixXd::Identity(t_dim + 1, t_dim + 1);
    pose_matrix.block(0, t_dim, t_dim, 1) = factor.measured();
    pose = T(pose_matrix);

    gtsam::Matrix covar_trans =
        factor_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel())
            ->covariance();

    // Large rotation covariance and given translation covar
    covariance_matrix = Eigen::MatrixXd::Identity(dim, dim) * 1.0e+08;
    covariance_matrix.block(r_dim, r_dim, t_dim, t_dim) = covar_trans;
  }

  /* method to combine two poses (along with their covariances) */
  /* ---------------------------------------------------------- */
  Pose::Ptr compose(const Pose::Ptr& other) const override {
    auto other_casted = dynamic_cast<PoseWithCovariance<T>*>(other.get());
    PoseWithCovariance<T> out;
    gtsam::Matrix Ha, Hb;

    out.pose = pose.compose(other_casted->pose, Ha, Hb);
    out.covariance_matrix = Ha * covariance_matrix * Ha.transpose() +
                            Hb * other_casted->covariance_matrix * Hb.transpose();

    if (!rotation_info || !other_casted->rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithCovariance>(out);
  }

  /* method to invert a pose along with its covariance -------- */
  /* ---------------------------------------------------------- */
  Pose::Ptr inverse() const override {
    PoseWithCovariance<T> out;
    out.pose = pose.inverse();
    out.covariance_matrix = covariance_matrix;
    if (!rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithCovariance>(out);
  }

  /* method to find the transform between two poses ------------ */
  /* ----------------------------------------------------------- */
  Pose::Ptr between(const Pose::Ptr& other) const override {
    auto other_casted = dynamic_cast<PoseWithCovariance<T>*>(other.get());
    PoseWithCovariance<T> out;
    gtsam::Matrix Ha, Hb;
    out.pose = pose.between(other_casted->pose, Ha, Hb);  // returns between in a frame
    out.covariance_matrix =
        other_casted->covariance_matrix - Ha * covariance_matrix * Ha.transpose();
    bool pos_semi_def = true;
    // compute the Cholesky decomp
    Eigen::LLT<Eigen::MatrixXd> lltCovar1(out.covariance_matrix);
    if (lltCovar1.info() == Eigen::NumericalIssue) {
      pos_semi_def = false;
    }

    if (!pos_semi_def) {
      other_casted->pose.between(pose, Ha, Hb);  // returns between in a frame
      out.covariance_matrix =
          covariance_matrix - Ha * other_casted->covariance_matrix * Ha.transpose();

      //// Check if positive semidef
      // Eigen::LLT<Eigen::MatrixXd> lltCovar2(out.covariance_matrix);
      // if (lltCovar2.info() == Eigen::NumericalIssue) {
      //  std::cout << "Warning: Covariance matrix between two poses not PSD\n";
      //}
    }
    if (!rotation_info || !other_casted->rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithCovariance>(out);
  }

  double mahalanobis_norm() const {
    // calculate mahalanobis norm
    gtsam::Vector log = T::Logmap(pose);
    if (!rotation_info) {
      // only use translation part
      int t_dim = getTranslationDim<T>();
      int r_dim = getRotationDim<T>();
      Eigen::MatrixXd cov_block = covariance_matrix.block(r_dim, r_dim, t_dim, t_dim);
      return std::sqrt(log.tail(t_dim).transpose() * cov_block.inverse() *
                       log.tail(t_dim));
    }

    return std::sqrt(log.transpose() * covariance_matrix.inverse() * log);
  }
};

/** \struct PoseWithNode
 *  \brief Structure to store a pose and its distance (number of nodes) from
 * start \currently supports gtsam::Pose2 and gtsam::Pose3
 */
template <class T>
struct PoseWithNode : Pose {
  using Ptr = std::unique_ptr<PoseWithNode>;
  /* variables ------------------------------------------------ */
  /* ---------------------------------------------------------- */
  T pose;                     // ex. gtsam::Pose3
  int node;                   // node away from prior
  bool rotation_info = true;  // to deal with no rotation info case

  /* default constructor -------------------------------------- */
  PoseWithNode() {
    pose = T();
    node = 0;
  }

  /* basic constructor ---------------------------------------- */
  PoseWithNode(T pose_in, int node_in) {
    pose = pose_in;
    node = node_in;
  }

  /* construct from gtsam prior factor ------------------------ */
  explicit PoseWithNode(const gtsam::PriorFactor<T>& prior_factor) {
    T value = prior_factor.prior();
    pose = value;
    node = 0;
  }

  /* construct from gtsam between factor  --------------------- */
  explicit PoseWithNode(const gtsam::BetweenFactor<T>& between_factor) {
    pose = between_factor.measured();
    node = 1;
  }

  /* construct from gtsam pose to point factor --------------- */
  template <typename pointT>
  explicit PoseWithNode(const gtsam::PoseToPointFactor<T, pointT>& factor) {
    const int t_dim = getTranslationDim<T>();
    T temp;
    gtsam::Matrix pose_matrix = temp.matrix();
    pose_matrix.block(0, t_dim, t_dim, 1) = factor.measured();
    pose = T(pose_matrix);

    node = 1;
  }

  /* method to combine two poses (along with their node numbers) */
  /* ---------------------------------------------------------- */
  Pose::Ptr compose(const Pose::Ptr& other) const override {
    auto other_casted = dynamic_cast<PoseWithNode<T>*>(other.get());
    PoseWithNode<T> out;

    out.pose = pose.compose(other_casted->pose);
    out.node = node + other_casted->node;

    if (!rotation_info || !other_casted->rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithNode>(out);
  }

  /* method to invert a pose along with its covariance -------- */
  /* ---------------------------------------------------------- */
  Pose::Ptr inverse() const override {
    PoseWithNode<T> out;

    out.pose = pose.inverse();
    out.node = node;

    if (!rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithNode>(out);
  }

  /* method to find the transform between two poses ------------ */
  /* ----------------------------------------------------------- */
  Pose::Ptr between(const Pose::Ptr& other) const override {
    auto other_casted = dynamic_cast<PoseWithNode<T>*>(other.get());
    PoseWithNode<T> out;
    out.pose = pose.between(other_casted->pose);  // returns between in a frame

    out.node = abs(other_casted->node - node);

    if (!rotation_info || !other_casted->rotation_info) out.rotation_info = false;
    return std::make_unique<PoseWithNode>(out);
  }

  double avg_trans_norm() const {
    // calculate mahalanobis norm
    gtsam::Vector log = T::Logmap(pose);
    const int t_dim = getTranslationDim<T>();
    return std::sqrt(log.tail(t_dim).transpose() * log.tail(t_dim)) / node;
  }

  double avg_rot_norm() const {
    // calculate mahalanobis norm
    if (!rotation_info) return 0;
    gtsam::Vector log = T::Logmap(pose);
    const int r_dim = getRotationDim<T>();
    return std::sqrt(log.head(r_dim).transpose() * log.head(r_dim)) / node;
  }
};

template <typename T, typename Ptr>
T* pose_pointer_cast(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get());
}

template <typename T, typename Ptr>
bool pose_is_underlying_type(Ptr& ptr) {
  return dynamic_cast<T*>(ptr.get()) != nullptr;
}

}  // namespace kimera_rpgo
