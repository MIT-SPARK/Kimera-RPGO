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
  AB.pose = poseAB;
  AB.covariance_matrix = Eigen::MatrixXd::Identity(6,6);

  B = A.compose(AB);

  EXPECT(gtsam::assert_equal(A.pose.compose(AB.pose), B.pose));

  Eigen::MatrixXd B_covar = Eigen::MatrixXd::Zero(6,6);
  B_covar.row(0) << 2, 0, 0, 0, -1, 1;
  B_covar.row(1) << 0, 2, 0, 1, 0, -1;
  B_covar.row(2) << 0, 0, 2, -1, 1, 0;
  B_covar.row(3) << 0, 1, -1, 4, -1, -1;
  B_covar.row(4) << -1, 0, 1, -1, 4, -1;
  B_covar.row(5) << 1, -1, 0, -1, -1, 4;

  EXPECT(gtsam::assert_equal(B.covariance_matrix, B_covar));

  // Then rotation only 
  gtsam::Pose3 poseBC(gtsam::Rot3(0,0,0,1), gtsam::Point3());
  BC.pose = poseBC;
  BC.covariance_matrix = Eigen::MatrixXd::Identity(6,6);
  BC.covariance_matrix.block(3,3,3,3) = 100 * Eigen::MatrixXd::Identity(3,3);

  C = B.compose(BC);

  EXPECT(gtsam::assert_equal(B.pose.compose(BC.pose), C.pose));

  Eigen::MatrixXd C_covar = Eigen::MatrixXd::Zero(6,6);
  C_covar.row(0) << 3, 0, 0, 0, -1, -1;
  C_covar.row(1) << 0, 3, 0, 1, 0, 1;
  C_covar.row(2) << 0, 0, 3, 1, -1, 0;
  C_covar.row(3) << 0, 1, 1, 104, -1, 1;
  C_covar.row(4) << -1, 0, -1, -1, 104, 1;
  C_covar.row(5) << -1, 1, 0, 1, 1, 104;

  EXPECT(gtsam::assert_equal(C.covariance_matrix, C_covar));

  // rotation and translation 
  gtsam::Pose3 poseCD(gtsam::Rot3(0,0,1,0), gtsam::Point3(1,0,0));
  CD.pose = poseCD;
  CD.covariance_matrix = Eigen::MatrixXd::Identity(6,6);

  D = C.compose(CD);

  EXPECT(gtsam::assert_equal(C.pose.compose(CD.pose), D.pose));

  Eigen::MatrixXd D_covar = Eigen::MatrixXd::Zero(6,6);
  D_covar.row(0) << 4, 0, 0, 0, 1, -1;
  D_covar.row(1) << 0, 4, 0, -1, 0, 2;
  D_covar.row(2) << 0, 0, 4, 1, -2, 0;
  D_covar.row(3) << 0, -1, 1, 105, 0, 0;
  D_covar.row(4) << 1, 0, -2, 0, 106, -1;
  D_covar.row(5) << -1, 2, 0, 0, -1, 106;

  EXPECT(gtsam::assert_equal(D.covariance_matrix, D_covar));

}

/* ************************************************************************* */
TEST(PoseWithCovariance, Between)
{
  // Test the between operator for the PoseWithCovariance struct 
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
