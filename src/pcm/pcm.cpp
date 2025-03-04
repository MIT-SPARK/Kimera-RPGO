#include "kimera_rpgo/pcm/pcm.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include "kimera_rpgo/utils/utils.h"

namespace kimera_rpgo {
using gtsam::BetweenFactor;
using gtsam::NonlinearFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::Point2;
using gtsam::Point3;
using gtsam::Pose2;
using gtsam::Pose3;
using gtsam::PoseToPointFactor;

std::ostream& operator<<(std::ostream& os, const PcmConfig::MaxCliqueMode& mode) {
  switch (mode) {
    case PcmConfig::MaxCliqueMode::EXACT:
      os << "EXACT\n";
      break;
    case PcmConfig::MaxCliqueMode::HEURISTIC:
      os << "HEURISTIC\n";
      break;
    case PcmConfig::MaxCliqueMode::INCREMENTAL:
      os << "INCREMENTAL\n";
      break;
  }
  return os;
}
std::ostream& operator<<(std::ostream& os, const PcmConfig& type) {
  os << "PCM Config\n";
  os << "  odom_check: " << type.odom_check;
  if (type.metric_type == PcmConfig::MetricType::NODE) {
    os << "\n  metric_type: NODE";
    os << "\n    trans_threshold: " << type.trans_threshold;
    os << "\n    rot_threshold: " << type.rot_threshold;
    if (type.odom_check) {
      os << "\n    odom_trans_threshold: " << type.odom_trans_threshold;
      os << "\n    odom_rot_threshold: " << type.odom_rot_threshold;
    }
  }

  if (type.metric_type == PcmConfig::MetricType::COVARIANCE) {
    os << "\n  metric_type: COVARIANCE";
    os << "\n    mahalanobis_threshold: " << type.mahalanobis_threshold;
    if (type.odom_check) {
      os << "\n    odom_mahalanobis_threshold: " << type.odom_mahalanobis_threshold;
    }
  }
  os << "\nmax_clique_mode: " << type.max_clique_mode << "\n";
  return os;
}

enum class MeasurementType { ODOM, LANDMARK, LOOPCLOSURE, OTHER };

MeasurementType getMeasurementType(const NonlinearFactor::shared_ptr& factor) {
  if (factor_is_underlying_type<BetweenFactor<Pose3>>(factor) ||
      factor_is_underlying_type<BetweenFactor<Pose2>>(factor)) {
    if (factor->front() + 1 == factor->back()) {
      return MeasurementType::ODOM;
    }
    return MeasurementType::LOOPCLOSURE;
  }
  if (factor_is_underlying_type<PoseToPointFactor<Pose3, Point3>>(factor) ||
      factor_is_underlying_type<PoseToPointFactor<Pose2, Point2>>(factor)) {
    return MeasurementType::LANDMARK;
  }
  return MeasurementType::OTHER;
}

Pcm::Pcm(const PcmConfig& config) : config_(config) {}
Pcm::~Pcm() {}

void Pcm::processBatch(const NonlinearFactorGraph& factors,
                       const std::set<size_t>& corrupted_odom_indices) {
  factors_.add(factors);
  for (size_t i = 0; i < factors.size(); ++i) {
    const auto& factor = factors.at(i);
    auto type = getMeasurementType(factor);
    // TODO(Yun) better way to do this?
    if (corrupted_odom_indices.count(i)) {
      std::cout << "Setting corrupted odom:\n";
      factor->print("\n");
      type = MeasurementType::LOOPCLOSURE;
      startNewOdomBackbone();
    }
    switch (type) {
      case MeasurementType::ODOM:
        odom_indices_.push_back(i);
        addOdometry(factor);
        break;
      case MeasurementType::OTHER:
        other_indices_.push_back(i);
        break;
      case MeasurementType::LOOPCLOSURE:
        addLoopClosure(factor, i);
        break;
      case MeasurementType::LANDMARK:
        addLandmarkMeasurement(factor, i);
        break;
    }
  }
  findInliers();
  buildInlierGraph();
}

void Pcm::processIncremental(const NonlinearFactorGraph& /*new_factors*/) {
  std::invalid_argument("Pcm::processIncremental not implemented");
}

void Pcm::addLoopClosure(const NonlinearFactor::shared_ptr& factor, size_t index) {
  if (config_.odom_check) {
    if (!checkOdomConsistent(factor)) {
      return;
    }
  }
  const size_t idx = loop_closures_.factors.size();
  loop_closures_.factors.add(factor);
  loop_closures_.indices.push_back(index);
  gtsam::Matrix new_adj_matrix = Eigen::MatrixXd::Zero(idx + 1, idx + 1);
  if (idx > 0) {
    new_adj_matrix.topLeftCorner(idx, idx) = loop_closures_.adj_matrix;
  }
  for (size_t other_idx = 0; other_idx < idx; ++other_idx) {
    if (checkPairwiseConsistent(factor, loop_closures_.factors[other_idx])) {
      new_adj_matrix(idx, other_idx) = 1;
      new_adj_matrix(other_idx, idx) = 1;
    }
  }
  loop_closures_.adj_matrix = new_adj_matrix;
}

void Pcm::addLandmarkMeasurement(const NonlinearFactor::shared_ptr& factor,
                                 size_t index) {
  gtsam::Key ldmk_key = factor->back();
  if (!landmarks_.count(ldmk_key)) {
    landmarks_[ldmk_key] = PcmMeasurements();
  }
  const size_t idx = landmarks_[ldmk_key].factors.size();
  landmarks_[ldmk_key].factors.add(factor);
  landmarks_[ldmk_key].indices.push_back(index);
  gtsam::Matrix new_adj_matrix = Eigen::MatrixXd::Zero(idx + 1, idx + 1);
  if (idx > 0) {
    new_adj_matrix.topLeftCorner(idx, idx) = landmarks_[ldmk_key].adj_matrix;
  }
  for (size_t other_idx = 0; other_idx < idx; ++other_idx) {
    if (checkLandmarkPairwiseConsistent(factor,
                                        landmarks_[ldmk_key].factors[other_idx])) {
      new_adj_matrix(idx, other_idx) = 1;
      new_adj_matrix(other_idx, idx) = 1;
    }
  }
  landmarks_[ldmk_key].adj_matrix = new_adj_matrix;
}

void Pcm::initOdometryPose2(const gtsam::Key& key) {
  if (config_.metric_type == PcmConfig::MetricType::NODE) {
    odometry_backbone_.rbegin()->second.poses[key] =
        std::make_unique<PoseWithNode<Pose2>>();
  } else {
    odometry_backbone_.rbegin()->second.poses[key] =
        std::make_unique<PoseWithCovariance<Pose2>>();
  }
}

void Pcm::initOdometryPose3(const gtsam::Key& key) {
  if (config_.metric_type == PcmConfig::MetricType::NODE) {
    odometry_backbone_.rbegin()->second.poses[key] =
        std::make_unique<PoseWithNode<Pose3>>();
  } else {
    odometry_backbone_.rbegin()->second.poses[key] =
        std::make_unique<PoseWithCovariance<Pose3>>();
  }
}

void Pcm::startNewOdomBackbone() { start_new_odom_backbone_ = true; }
void Pcm::addOdometry(const NonlinearFactor::shared_ptr& factor) {
  const auto& prev = factor->front();
  const auto& curr = factor->back();

  if (start_new_odom_backbone_) {
    odometry_backbone_[prev] = Trajectory();
    start_new_odom_backbone_ = false;
  }

  auto& current_backbone = odometry_backbone_.rbegin()->second;
  if (current_backbone.poses.size() == 0) {
    if (factor_is_underlying_type<BetweenFactor<Pose3>>(factor)) {
      initOdometryPose3(prev);
    } else {
      initOdometryPose2(prev);
    }
  }
  auto odom_delta = getBetweenFactorPose(factor);
  current_backbone.poses[curr] = current_backbone.poses.at(prev)->compose(odom_delta);
}

Pose::Ptr Pcm::getBetweenFactorPose(const NonlinearFactor::shared_ptr& factor) const {
  if (factor_is_underlying_type<BetweenFactor<Pose3>>(factor)) {
    const auto& between_3d = *factor_pointer_cast<BetweenFactor<Pose3>>(factor);
    if (config_.metric_type == PcmConfig::MetricType::NODE) {
      return std::make_unique<PoseWithNode<Pose3>>(between_3d);
    } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
      return std::make_unique<PoseWithCovariance<Pose3>>(between_3d);
    } else {
      std::invalid_argument("Unknown PCM metric type");
      return nullptr;
    }
  } else {
    const auto& between_2d = *factor_pointer_cast<BetweenFactor<Pose2>>(factor);
    if (config_.metric_type == PcmConfig::MetricType::NODE) {
      return std::make_unique<PoseWithNode<Pose2>>(between_2d);
    } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
      return std::make_unique<PoseWithCovariance<Pose2>>(between_2d);
    } else {
      std::invalid_argument("Unknown PCM metric type");
      return nullptr;
    }
  }
}

Pose::Ptr Pcm::getLandmarkFactorPose(const NonlinearFactor::shared_ptr& factor) const {
  if (factor_is_underlying_type<PoseToPointFactor<Pose3, Point3>>(factor)) {
    const auto& ldmk_meas_3d = *factor_pointer_cast<PoseToPointFactor<Pose3>>(factor);
    if (config_.metric_type == PcmConfig::MetricType::NODE) {
      return std::make_unique<PoseWithNode<Pose3>>(ldmk_meas_3d);
    } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
      return std::make_unique<PoseWithCovariance<Pose3>>(ldmk_meas_3d);
    } else {
      std::invalid_argument("Unknown PCM metric type");
      return nullptr;
    }
  } else {
    const auto& ldmk_meas_2d =
        *factor_pointer_cast<PoseToPointFactor<Pose2, Point2>>(factor);
    if (config_.metric_type == PcmConfig::MetricType::NODE) {
      return std::make_unique<PoseWithNode<Pose2>>(ldmk_meas_2d);
    } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
      return std::make_unique<PoseWithCovariance<Pose2>>(ldmk_meas_2d);
    } else {
      std::invalid_argument("Unknown PCM metric type");
      return nullptr;
    }
  }
}

bool Pcm::checkOdomThreshold(const Pose::Ptr& error) const {
  if (config_.metric_type == PcmConfig::MetricType::NODE) {
    if (pose_is_underlying_type<PoseWithNode<Pose2>>(error)) {
      auto pose_with_node = pose_pointer_cast<PoseWithNode<Pose2>>(error);
      return (pose_with_node->avg_trans_norm() < config_.odom_trans_threshold &&
              pose_with_node->avg_rot_norm() < config_.odom_rot_threshold);
    }
    auto pose_with_node = pose_pointer_cast<PoseWithNode<Pose3>>(error);
    return (pose_with_node->avg_trans_norm() < config_.odom_trans_threshold &&
            pose_with_node->avg_rot_norm() < config_.odom_rot_threshold);
  } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
    if (pose_is_underlying_type<PoseWithCovariance<Pose2>>(error)) {
      auto pose_with_cov = pose_pointer_cast<PoseWithCovariance<Pose2>>(error);
      return pose_with_cov->mahalanobis_norm() < config_.odom_mahalanobis_threshold;
    }
    auto pose_with_cov = pose_pointer_cast<PoseWithCovariance<Pose3>>(error);
    return pose_with_cov->mahalanobis_norm() < config_.odom_mahalanobis_threshold;
  } else {
    std::invalid_argument("Unknown PCM metric type");
    return false;
  }
}

bool Pcm::checkThreshold(const Pose::Ptr& error) const {
  if (config_.metric_type == PcmConfig::MetricType::NODE) {
    if (pose_is_underlying_type<PoseWithNode<Pose2>>(error)) {
      auto pose_with_node = pose_pointer_cast<PoseWithNode<Pose2>>(error);
      return (pose_with_node->avg_trans_norm() < config_.trans_threshold &&
              pose_with_node->avg_rot_norm() < config_.rot_threshold);
    }
    auto pose_with_node = pose_pointer_cast<PoseWithNode<Pose3>>(error);
    return (pose_with_node->avg_trans_norm() < config_.trans_threshold &&
            pose_with_node->avg_rot_norm() < config_.rot_threshold);
  } else if (config_.metric_type == PcmConfig::MetricType::COVARIANCE) {
    if (pose_is_underlying_type<PoseWithCovariance<Pose2>>(error)) {
      auto pose_with_cov = pose_pointer_cast<PoseWithCovariance<Pose2>>(error);
      return pose_with_cov->mahalanobis_norm() < config_.mahalanobis_threshold;
    }
    auto pose_with_cov = pose_pointer_cast<PoseWithCovariance<Pose3>>(error);
    return pose_with_cov->mahalanobis_norm() < config_.mahalanobis_threshold;
  } else {
    std::invalid_argument("Unknown PCM metric type");
    return false;
  }
}

Pose::Ptr Pcm::getOdomBackbone(const gtsam::Key& key_i, const gtsam::Key& key_j) const {
  // Try get odom backbone measurement between i and j
  // Return false if no backbone exists between the two
  auto i_upper = odometry_backbone_.upper_bound(key_i);
  auto j_upper = odometry_backbone_.upper_bound(key_j);
  if (i_upper != j_upper) {
    return nullptr;
  }

  return std::prev(i_upper)->second.getBetween(key_i, key_j);
}

bool Pcm::checkOdomConsistent(const NonlinearFactor::shared_ptr& factor) const {
  const auto& key_i = factor->front();
  const auto& key_j = factor->back();
  Pose::Ptr pij_odom = getOdomBackbone(key_i, key_j);
  if (pij_odom) {
    // No odometry backbone, so ignore check. TODO(Yun) print message?
    return true;
  }
  Pose::Ptr pij_lc = getBetweenFactorPose(factor);
  Pose::Ptr loop = pij_odom->compose(pij_lc->inverse());
  return checkThreshold(loop);
}

bool Pcm::checkPairwiseConsistent(const NonlinearFactor::shared_ptr& factor_ij,
                                  const NonlinearFactor::shared_ptr& factor_kl) const {
  auto key_i = factor_ij->front();
  auto key_j = factor_ij->back();
  auto key_k = factor_kl->front();
  auto key_l = factor_kl->back();

  bool invert_kl = false;
  if (abs(static_cast<int>(key_l) - static_cast<int>(key_i)) >
      abs(static_cast<int>(key_k) - static_cast<int>(key_i))) {
    auto temp_key_k = key_l;
    key_l = key_k;
    key_k = temp_key_k;
    invert_kl = true;
  }

  Pose::Ptr pli_odom = getOdomBackbone(key_l, key_i);
  if (!pli_odom) {
    // TODO(Yun) print message
    return true;
  }

  Pose::Ptr pjk_odom = getOdomBackbone(key_j, key_k);
  if (!pjk_odom) {
    // TODO(Yun) print message
    return true;
  }
  Pose::Ptr pij_lc = getBetweenFactorPose(factor_ij);
  Pose::Ptr pkl_lc = getBetweenFactorPose(factor_kl);
  if (invert_kl) {
    pkl_lc = pkl_lc->inverse();
  }
  Pose::Ptr loop = pij_lc->compose(pjk_odom)->compose(pkl_lc)->compose(pli_odom);
  return checkThreshold(loop);
}

bool Pcm::checkLandmarkPairwiseConsistent(
    const NonlinearFactor::shared_ptr& factor_il,
    const NonlinearFactor::shared_ptr& factor_jl) const {
  auto key_i = factor_il->front();
  auto key_j = factor_jl->front();

  Pose::Ptr pij_odom = getOdomBackbone(key_i, key_j);
  if (!pij_odom) {
    // TODO(Yun) print message
    return true;
  }

  Pose::Ptr pil_ldmk = getLandmarkFactorPose(factor_il);
  Pose::Ptr pjl_ldmk = getLandmarkFactorPose(factor_jl);
  Pose::Ptr loop = pil_ldmk->between(pij_odom->compose(pjl_ldmk));
  return checkThreshold(loop);
}

size_t Pcm::callMaxClique(const gtsam::Matrix& adj_matrix,
                          std::vector<int>& inliers) const {
  switch (config_.max_clique_mode) {
    case (PcmConfig::MaxCliqueMode::EXACT):
      return findMaxClique(adj_matrix, inliers);
    case (PcmConfig::MaxCliqueMode::HEURISTIC):
      return findMaxCliqueHeu(adj_matrix, inliers);
    case (PcmConfig::MaxCliqueMode::INCREMENTAL):
      std::invalid_argument("PCM incremental max clique not yet implemented");
      return 0;
    default:
      std::invalid_argument("PCM invalid max clique mode");
      return 0;
  }
}

void Pcm::findInliers() {
  // Reset consistent factors
  loop_closures_.consistent_factors = NonlinearFactorGraph();
  std::vector<int> inlier_indices;
  callMaxClique(loop_closures_.adj_matrix, inlier_indices);
  for (const auto& idx : inlier_indices) {
    // TODO(Yun) might be able to get rid of consistent_factors and factors and just
    // keep indices
    loop_closures_.consistent_factors.add(loop_closures_.factors[idx]);
    loop_closures_.consistent_indices.push_back(loop_closures_.indices[idx]);
  }

  for (auto& ldmk_meas : landmarks_) {
    ldmk_meas.second.consistent_factors = NonlinearFactorGraph();
    std::vector<int> inlier_indices;
    callMaxClique(ldmk_meas.second.adj_matrix, inlier_indices);
    for (const auto& idx : inlier_indices) {
      ldmk_meas.second.consistent_factors.add(ldmk_meas.second.factors[idx]);
      ldmk_meas.second.consistent_indices.push_back(ldmk_meas.second.indices[idx]);
    }
  }
}

void Pcm::buildInlierGraph() {
  inlier_indices_ = std::vector<size_t>();

  for (const auto& ind : odom_indices_) {
    inlier_indices_.push_back(ind);
  }

  for (const auto& ind : other_indices_) {
    inlier_indices_.push_back(ind);
  }

  inlier_indices_.insert(inlier_indices_.end(),
                         loop_closures_.consistent_indices.begin(),
                         loop_closures_.consistent_indices.end());
  for (const auto& ldmk_meas : landmarks_) {
    inlier_indices_.insert(inlier_indices_.end(),
                           ldmk_meas.second.consistent_indices.begin(),
                           ldmk_meas.second.consistent_indices.end());
  }

  result_ = NonlinearFactorGraph();
  for (const auto& idx : inlier_indices_) {
    result_.add(factors_[idx]);
  }
}
}  // namespace kimera_rpgo
