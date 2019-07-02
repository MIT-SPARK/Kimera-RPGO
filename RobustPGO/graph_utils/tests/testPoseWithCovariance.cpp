/**
 * @file    testPoseWithCovariance.cpp
 * @brief   Unit test for PoseWithCovariance and calculations 
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>

#include "RobustPGO/graph_utils/graph_utils.h" 

/* ************************************************************************* */
TEST(PoseWithCovariance, Inverse)
{
  // Test the inverse operator for PoseWithCovariance struct 

  graph_utils::PoseWithCovariance<gtsam::Pose3> A, B; 

  // Create linearization points
  gtsam::Pose3 poseA(gtsam::Rot3(), gtsam::Point3(0, 0, 0));

  A.pose = poseA; 
  A.covariance_matrix = Eigen::MatrixXd::Identity(6,6);

  B = A.inverse();

  EXPECT(gtsam::assert_equal(B.pose, A.pose.inverse()));

  EXPECT(gtsam::assert_equal(B.covariance_matrix, A.covariance_matrix));
}

/* ************************************************************************* */
TEST(PoseWithCovariance, Compose)
{
  // Test the compose operator for PoseWithCovariance struct 
  graph_utils::PoseWithCovariance<gtsam::Pose3> A, AB, B, BC, C, CD, D; 

  A.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,1,1)); // start
  A.covariance_matrix = Eigen::MatrixXd::Identity(6,6);

  // First test a translation only 
  gtsam::Pose3 poseAB(gtsam::Rot3(), gtsam::Point3(1,1,1));
  AB.covariance_matrix = Eigen::MatrixXd::Identity(6,6);

  B = A.compose(AB);

  EXPECT(gtsam::assert_equal(A.pose.compose(AB.pose), B.pose));

  Eigen::MatrixXd B_covar = Eigen::MatrixXd::Zero(6,6);

  std::cout << B.covariance_matrix << std::endl; 

  EXPECT(gtsam::assert_equal(B.covariance_matrix, B_covar));

}

/* ************************************************************************* */
TEST(PoseWithCovariance, Between)
{
  // Test the between operator for the PoseWithCovariance struct 
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
