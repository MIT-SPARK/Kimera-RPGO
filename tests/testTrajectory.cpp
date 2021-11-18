/**
 * @file    testTrajectory.cpp
 * @brief   Unit test for long trajectory
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "KimeraRPGO/utils/GeometryUtils.h"
#include "KimeraRPGO/utils/GraphUtils.h"

using KimeraRPGO::PoseWithCovariance;
using KimeraRPGO::PoseWithNode;
using KimeraRPGO::Trajectory;

struct normal_rv {
  explicit normal_rv(Eigen::MatrixXd const& covar) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::MatrixXd transform;

  Eigen::VectorXd operator()() const {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;
    return transform * Eigen::VectorXd{transform.rows()}.unaryExpr(
                           [&](double x) { return dist(gen); });
  }
};

/* ************************************************************************* */
TEST(Trajectory, MonteCarlo) {
  gtsam::Key start_id = 0;
  gtsam::Key end_id = 100;
  gtsam::Matrix init_covar = Eigen::MatrixXd::Zero(6, 6);  // initialize as zero

  // construct initial pose with covar
  PoseWithCovariance<gtsam::Pose3> initial_pose;
  initial_pose.pose = gtsam::Pose3();
  initial_pose.covariance_matrix = init_covar;

  // populate
  Trajectory<gtsam::Pose3, PoseWithCovariance> test_traj;
  test_traj.poses[start_id] = initial_pose;

  PoseWithCovariance<gtsam::Pose3> current_pose = initial_pose;
  for (gtsam::Key i = start_id; i < end_id; i++) {
    // rotation and translation
    PoseWithCovariance<gtsam::Pose3> odom;
    gtsam::Pose3 tf(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1, 1, 0));
    odom.pose = tf;
    odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6, 6);

    current_pose = current_pose.compose(odom);  // update pose
    test_traj.poses[i] = current_pose;
  }

  // Now check the between of 2 and 99
  PoseWithCovariance<gtsam::Pose3> between_pose = test_traj.getBetween(2, 99);

  // Now compare this to both the stiched together result
  // and monte carlo result
  PoseWithCovariance<gtsam::Pose3> between_rebuild;
  between_rebuild.pose = gtsam::Pose3();
  between_rebuild.covariance_matrix = Eigen::MatrixXd::Zero(6, 6);

  for (gtsam::Key i = 2; i < 99; i++) {
    // rotation and translation
    PoseWithCovariance<gtsam::Pose3> odom;
    gtsam::Pose3 tf(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1, 1, 0));
    odom.pose = tf;
    odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6, 6);
    // between_rebuild.pose.print();
    between_rebuild = between_rebuild.compose(odom);  // update pose (stich)
  }

  // monte carlo
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6, 6);
  for (size_t i = 0; i < 1000; i++) {
    gtsam::Pose3 between_mc = gtsam::Pose3();

    for (gtsam::Key i = 2; i < 99; i++) {
      // rotation and translation
      PoseWithCovariance<gtsam::Pose3> odom;
      gtsam::Pose3 tf(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1, 1, 0));
      odom.pose = tf;
      odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6, 6);

      normal_rv noiseOdom(odom.covariance_matrix);
      gtsam::Pose3 odom_mc = odom.pose.expmap(noiseOdom());
      between_mc = between_mc.compose(odom_mc);
    }

    gtsam::Vector6 eps =
        gtsam::Pose3::Logmap(between_rebuild.pose.inverse() * between_mc);
    cov = cov + eps * eps.transpose();
  }

  cov = cov / 1000.0;

  EXPECT(gtsam::assert_equal(between_pose.pose, between_rebuild.pose));
  EXPECT(gtsam::assert_equal(between_pose.covariance_matrix,
                             between_rebuild.covariance_matrix));
  // error grows as covariance grows (as trajectory gets long)
  EXPECT(gtsam::assert_equal(between_rebuild.covariance_matrix, cov, 10));
}

/* ************************************************************************* */
TEST(Trajectory, PoseWithCovarianceBetween) {
  gtsam::Key a0 = gtsam::Symbol('a', 0);
  gtsam::Key b0 = gtsam::Symbol('b', 0);
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key b1 = gtsam::Symbol('b', 2);

  // define a0 and b0
  PoseWithCovariance<gtsam::Pose3> pose_a0, pose_b0, pose_a1, pose_b1,
      pose_a0b0, pose_a01, pose_b01;
  pose_a0.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1));
  pose_a0.covariance_matrix = Eigen::MatrixXd::Zero(6, 6);

  pose_a0b0.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_a0b0.covariance_matrix = 0.01 * Eigen::MatrixXd::Identity(6, 6);
  pose_b0 = pose_a0.compose(pose_a0b0);

  // construct odometry to a1 and b1
  pose_a01.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0));
  pose_a01.covariance_matrix = 0.1 * Eigen::MatrixXd::Identity(6, 6);

  gtsam::Matrix3 R;
  R.row(0) << 0, -1, 0;
  R.row(1) << 1, 0, 0;
  R.row(2) << 0, 0, 1;
  pose_b01.pose = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
  pose_b01.covariance_matrix = 0.2 * Eigen::MatrixXd::Identity(6, 6);

  // construct a1 and b1
  pose_a1 = pose_a0.compose(pose_a01);
  pose_b1 = pose_b0.compose(pose_b01);

  // add to trajectory
  Trajectory<gtsam::Pose3, PoseWithCovariance> test_traj;
  test_traj.poses[a0] = pose_a0;
  test_traj.poses[a1] = pose_a1;
  test_traj.poses[b0] = pose_b0;
  test_traj.poses[b1] = pose_b1;

  gtsam::Pose3 expected_between =
      gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(2, -1, 0));
  // Eigen::MatrixXd expected_covar = Eigen::MatrixXd::Zero(6,6); // TODO (Yun)
  // fix this expected_covar.row(0) << 0.41, -0.1, 0, 0, 0, 0.1;
  // expected_covar.row(1) << -0.1, 0.42, 0, 0, 0, -0.11;
  // expected_covar.row(2) << 0, 0, 0.52, -0.1, 0.11, 0;
  // expected_covar.row(3) << 0, 0, -0.1, 0.31, 0, 0;
  // expected_covar.row(4) << 0, 0, 0.11, 0, 0.31, 0;
  // expected_covar.row(5) << 0.1, -0.11, 0, 0, 0, 0.31;

  EXPECT(
      gtsam::assert_equal(expected_between, test_traj.getBetween(a1, b1).pose));
  // EXPECT(gtsam::assert_equal(expected_covar,
  // test_traj.getBetween(a1,b1).covariance_matrix));
}

/* ************************************************************************* */
TEST(Trajectory, PoseWithNodeBetween) {
  gtsam::Key a0 = gtsam::Symbol('a', 0);
  gtsam::Key b0 = gtsam::Symbol('b', 0);
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key b1 = gtsam::Symbol('b', 2);

  // define a0 and b0
  PoseWithNode<gtsam::Pose3> pose_a0, pose_b0, pose_a1, pose_b1, pose_a0b0,
      pose_a01, pose_b01;
  pose_a0.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1));
  pose_a0.node = 0;

  pose_a0b0.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  pose_a0b0.node = 1;
  pose_b0 = pose_a0.compose(pose_a0b0);

  // construct odometry to a1 and b1
  pose_a01.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0));
  pose_a01.node = 1;

  gtsam::Matrix3 R;
  R.row(0) << 0, -1, 0;
  R.row(1) << 1, 0, 0;
  R.row(2) << 0, 0, 1;
  pose_b01.pose = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
  pose_b01.node = 1;

  // construct a1 and b1
  pose_a1 = pose_a0.compose(pose_a01);
  pose_b1 = pose_b0.compose(pose_b01);

  // add to trajectory
  Trajectory<gtsam::Pose3, PoseWithNode> test_traj;
  test_traj.poses[a0] = pose_a0;
  test_traj.poses[a1] = pose_a1;
  test_traj.poses[b0] = pose_b0;
  test_traj.poses[b1] = pose_b1;

  gtsam::Pose3 expected_between =
      gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(2, -1, 0));

  EXPECT(
      gtsam::assert_equal(expected_between, test_traj.getBetween(a1, b1).pose));
  EXPECT(3 == test_traj.getBetween(a1, b1).node);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
