/**
 * @file    testTrajectory.cpp
 * @brief   Unit test for long trajectory
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "RobustPGO/utils/geometry_utils.h"
#include "RobustPGO/utils/graph_utils.h"

using namespace RobustPGO;

struct normal_rv {
  normal_rv(Eigen::MatrixXd const& covar) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
        eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::MatrixXd transform;

  Eigen::VectorXd operator()() const {
    static std::mt19937 gen { std::random_device{}() };
    static std::normal_distribution<> dist;
    return transform * Eigen::VectorXd{ transform.rows() }.unaryExpr([&](double x) {
      return dist(gen);
    });
  }
};

/* ************************************************************************* */
TEST(PoseWithCovariance, Trajectory)
{
  gtsam::Key start_id = 0;
  gtsam::Key end_id = 100;
  gtsam::Matrix init_covar = Eigen::MatrixXd::Zero(6,6); // initialize as zero

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
    gtsam::Pose3 tf(gtsam::Rot3(1,0,0,0), gtsam::Point3(1,1,0));
    odom.pose = tf;
    odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6,6);

    current_pose = current_pose.compose(odom); // update pose
    test_traj.poses[i] = current_pose;
  }

  // Now check the between of 2 and 99
  PoseWithCovariance<gtsam::Pose3> between_pose =
      test_traj.getBetween(2, 99);

  // Now compare this to both the stiched together result
  // and monte carlo result
  PoseWithCovariance<gtsam::Pose3> between_rebuild;
  between_rebuild.pose = gtsam::Pose3();
  between_rebuild.covariance_matrix = Eigen::MatrixXd::Zero(6,6);

  for (gtsam::Key i = 2; i < 99; i++) {
    // rotation and translation
    PoseWithCovariance<gtsam::Pose3> odom;
    gtsam::Pose3 tf(gtsam::Rot3(1,0,0,0), gtsam::Point3(1,1,0));
    odom.pose = tf;
    odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6,6);
    // between_rebuild.pose.print();
    between_rebuild = between_rebuild.compose(odom); // update pose (stich)
  }

  // monte carlo
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6,6);
  for (size_t i = 0; i < 1000; i++) {
    gtsam::Pose3 between_mc = gtsam::Pose3();

    for (gtsam::Key i = 2; i < 99; i++) {
      // rotation and translation
      PoseWithCovariance<gtsam::Pose3> odom;
      gtsam::Pose3 tf(gtsam::Rot3(1,0,0,0), gtsam::Point3(1,1,0));
      odom.pose = tf;
      odom.covariance_matrix = 0.0001 * Eigen::MatrixXd::Identity(6,6);

      normal_rv noiseOdom(odom.covariance_matrix);
      gtsam::Pose3 odom_mc = odom.pose.expmap(noiseOdom());
      between_mc = between_mc.compose(odom_mc);
    }

    gtsam::Vector6 eps = gtsam::Pose3::Logmap(between_rebuild.pose.inverse() * between_mc);
    cov = cov + eps * eps.transpose();
  }

  cov = cov/1000.0;

  EXPECT(gtsam::assert_equal(between_pose.pose, between_rebuild.pose));
  EXPECT(gtsam::assert_equal(between_pose.covariance_matrix, between_rebuild.covariance_matrix));
  // error grows as covariance grows (as trajectory gets long)
  EXPECT(gtsam::assert_equal(between_rebuild.covariance_matrix, cov, 10));

}



/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
