/**
 * @file    testPoseWithCovariance.cpp
 * @brief   Unit test for PoseWithCovariance and calculations 
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "RobustPGO/graph_utils/graph_utils.h" 

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
  A.covariance_matrix = 0.1 * Eigen::MatrixXd::Identity(6,6);
  A.covariance_matrix.block(0,0,3,3) = Eigen::MatrixXd::Zero(3,3);

  // First test a translation only 
  gtsam::Pose3 poseAB(gtsam::Rot3(), gtsam::Point3(1,1,1));
  AB.pose = poseAB;
  AB.covariance_matrix = 0.1 * Eigen::MatrixXd::Identity(6,6);
  AB.covariance_matrix.block(0,0,3,3) = Eigen::MatrixXd::Zero(3,3);

  B = A.compose(AB);

  EXPECT(gtsam::assert_equal(A.pose.compose(AB.pose), B.pose));

  // then test with monte carlo result (would take a while to compute)
  size_t sample_size = 1000;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6,6);
  for (size_t i = 0; i < sample_size; i++) {
    normal_rv noiseA(A.covariance_matrix);
    gtsam::Pose3 A_mc = A.pose.expmap(noiseA());
    normal_rv noiseAB(AB.covariance_matrix);
    gtsam::Pose3 AB_mc = AB.pose.expmap(noiseAB());
    gtsam::Pose3 B_mc = A_mc.compose(AB_mc);

    gtsam::Vector6 eps = gtsam::Pose3::Logmap(B.pose.inverse() * B_mc);
    cov = cov + eps * eps.transpose();
  }
  cov = cov/sample_size;
  EXPECT(gtsam::assert_equal(cov, B.covariance_matrix, 0.1));

  // Then rotation only 
  gtsam::Pose3 poseBC(gtsam::Rot3(1,0,0,0), gtsam::Point3());
  BC.pose = poseBC;
  BC.covariance_matrix = Eigen::MatrixXd::Zero(6,6);
  BC.covariance_matrix.block(3,3,3,3) = 0.01 * Eigen::MatrixXd::Identity(3,3);

  C = B.compose(BC);

  EXPECT(gtsam::assert_equal(B.pose.compose(BC.pose), C.pose));

  // then test with monte carlo result (would take a while to compute)
  cov = Eigen::MatrixXd::Zero(6,6);
  for (size_t i = 0; i < sample_size; i++) {
    normal_rv noiseB(B.covariance_matrix);
    gtsam::Pose3 B_mc = B.pose.expmap(noiseB());
    normal_rv noiseBC(BC.covariance_matrix);
    gtsam::Pose3 BC_mc = BC.pose.expmap(noiseBC());
    gtsam::Pose3 C_mc = B_mc.compose(BC_mc);

    gtsam::Vector6 eps = gtsam::Pose3::Logmap(C.pose.inverse() * C_mc);
    cov = cov + eps * eps.transpose();
  }
  cov = cov/sample_size;
  EXPECT(gtsam::assert_equal(cov, C.covariance_matrix, 0.1));

  // rotation and translation 
  gtsam::Pose3 poseCD(gtsam::Rot3(1,0,0,0), gtsam::Point3(1,0,0));
  CD.pose = poseCD;
  CD.covariance_matrix = 0.1 * Eigen::MatrixXd::Identity(6,6);
  CD.covariance_matrix.block(0,0,3,3) = Eigen::MatrixXd::Zero(3,3);

  D = C.compose(CD);

  EXPECT(gtsam::assert_equal(C.pose.compose(CD.pose), D.pose));

  // then test with monte carlo result (would take a while to compute)
  cov = Eigen::MatrixXd::Zero(6,6);
  for (size_t i = 0; i < sample_size; i++) {
    normal_rv noiseC(C.covariance_matrix);
    gtsam::Pose3 C_mc = C.pose.expmap(noiseC());
    normal_rv noiseCD(CD.covariance_matrix);
    gtsam::Pose3 CD_mc = CD.pose.expmap(noiseCD());
    gtsam::Pose3 D_mc = C_mc.compose(CD_mc);

    gtsam::Vector6 eps = gtsam::Pose3::Logmap(D.pose.inverse() * D_mc);
    cov = cov + eps * eps.transpose();
  }
  cov = cov/sample_size;
  EXPECT(gtsam::assert_equal(cov, D.covariance_matrix, 0.1));


}

/* ************************************************************************* */
TEST(PoseWithCovariance, Between)
{
  // Test the between operator for the PoseWithCovariance struct
  graph_utils::PoseWithCovariance<gtsam::Pose3> A, B, C;

  A.pose = gtsam::Pose3();

  Eigen::MatrixXd A_covar = Eigen::MatrixXd::Zero(6,6);
  A_covar.row(3) << 0, 0, 0, 0.41, 0, 0;
  A_covar.row(4) << 0, 0, 0, 0, 0.41, 0;
  A_covar.row(5) << 0, 0, 0, 0, 0, 0.41;
  A.covariance_matrix = A_covar; 

  C.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,0,0));

  Eigen::MatrixXd C_covar = Eigen::MatrixXd::Zero(6,6);
  C_covar.row(3) << 0, 0, 0, 0.51, 0, 0;
  C_covar.row(4) << 0, 0, 0, 0, 0.61, 0;
  C_covar.row(5) << 0, 0, 0, 0, 0, 0.61;
  C.covariance_matrix = C_covar;

  B = A.between(C);
  gtsam::Pose3 B_pose = gtsam::Pose3(gtsam::Rot3(1,0,0,0), gtsam::Point3(1,0,0));
  EXPECT(gtsam::assert_equal(B.pose, B.pose));
  // check with monte carlo result 
  size_t sample_size = 1000;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(6,6);
  for (size_t i = 0; i < sample_size; i++) {
    normal_rv noiseA(A.covariance_matrix);
    gtsam::Pose3 A_mc = A.pose.expmap(noiseA());
    normal_rv noiseB(B.covariance_matrix);
    gtsam::Pose3 B_mc = B.pose.expmap(noiseB());
    gtsam::Pose3 C_mc = A_mc.compose(B_mc);

    gtsam::Vector6 eps = gtsam::Pose3::Logmap(C.pose.inverse() * C_mc);
    cov = cov + eps * eps.transpose();
  }
  cov = cov/sample_size;
  EXPECT(gtsam::assert_equal(cov, C.covariance_matrix, 0.1));
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
