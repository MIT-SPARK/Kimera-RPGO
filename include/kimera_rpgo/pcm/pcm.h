#pragma once
#include "kimera_rpgo/pcm/graph_utils.h"

namespace kimera_rpgo {
struct PcmConfig {
  bool odom_check = false;

  double trans_threshold = 0.1;
  double rot_threshold = 0.01;
  double mahalanobis_threshold = 1.0;

  double odom_trans_threshold = 0.5;
  double odom_rot_threshold = 0.1;
  double odom_mahalanobis_threshold = 5.0;

  enum class MaxCliqueMode {
    EXACT,
    HEURISTIC,
    INCREMENTAL
  } max_clique_mode = MaxCliqueMode::HEURISTIC;

  enum class MetricType { NODE, COVARIANCE } metric_type = MetricType::NODE;
};

std::ostream& operator<<(std::ostream& os,
                         const PcmConfig::MaxCliqueMode& mode);

std::ostream& operator<<(std::ostream& os, const PcmConfig& type);

struct PcmMeasurements {
  gtsam::NonlinearFactorGraph factors;
  gtsam::NonlinearFactorGraph consistent_factors;
  gtsam::Matrix adj_matrix;
  std::vector<size_t> indices;
  std::vector<size_t> consistent_indices;
};

class Pcm {
 public:
  Pcm(const PcmConfig& config);
  ~Pcm();

  void processBatch(
      const gtsam::NonlinearFactorGraph& factors,
      const std::set<size_t>& corrupted_odom_indices = std::set<size_t>{});

  void processIncremental(const gtsam::NonlinearFactorGraph& new_factors);

  inline const gtsam::NonlinearFactorGraph& getInlierGraph() const {
    return result_;
  }

  inline const std::vector<size_t>& getInlierIndices() const {
    return inlier_indices_;
  }

 private:
  void addLoopClosure(const gtsam::NonlinearFactor::shared_ptr& factor,
                      size_t index);

  void addLandmarkMeasurement(const gtsam::NonlinearFactor::shared_ptr& factor,
                              size_t index);

  Pose::Ptr getBetweenFactorPose(
      const gtsam::NonlinearFactor::shared_ptr& factor) const;

  Pose::Ptr getLandmarkFactorPose(
      const gtsam::NonlinearFactor::shared_ptr& factor) const;

  void initOdometryPose2(const gtsam::Key& key);

  void initOdometryPose3(const gtsam::Key& key);

  void startNewOdomBackbone();

  void addOdometry(const gtsam::NonlinearFactor::shared_ptr& factor);

  Pose::Ptr getOdomBackbone(const gtsam::Key& key_i,
                            const gtsam::Key& key_j) const;

  bool checkOdomThreshold(const Pose::Ptr& error) const;

  bool checkThreshold(const Pose::Ptr& error) const;

  bool checkOdomConsistent(
      const gtsam::NonlinearFactor::shared_ptr& factor) const;

  bool checkPairwiseConsistent(
      const gtsam::NonlinearFactor::shared_ptr& factor_ij,
      const gtsam::NonlinearFactor::shared_ptr& factor_kl) const;

  bool checkLandmarkPairwiseConsistent(
      const gtsam::NonlinearFactor::shared_ptr& factor_il,
      const gtsam::NonlinearFactor::shared_ptr& factor_jl) const;

  size_t callMaxClique(const gtsam::Matrix& adj_matrix,
                       std::vector<int>& inliers) const;

  void findInliers();

  void buildInlierGraph();

 private:
  PcmConfig config_;

  // Book-keeping
  std::list<size_t> odom_indices_;
  std::list<size_t> other_indices_;

  // maps first key to the stable backbone
  // TODO(Yun) maybe write odom backbone class?
  std::map<gtsam::Key, Trajectory> odometry_backbone_;
  bool start_new_odom_backbone_ = true;

  PcmMeasurements loop_closures_;

  std::map<gtsam::Key, PcmMeasurements> landmarks_;

  gtsam::NonlinearFactorGraph factors_;

  gtsam::NonlinearFactorGraph result_;
  std::vector<size_t> inlier_indices_;
};
}  // namespace kimera_rpgo
