/**
 * @file    testPoseWithNode.cpp
 * @brief   Unit test for PoseWithNode and calculations
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <limits>
#include <random>

#include "KimeraRPGO/utils/GeometryUtils.h"

using KimeraRPGO::PoseWithNode;

/* ************************************************************************* */
TEST(PoseWithNode, Inverse) {
  // Test the inverse operator for PoseWithNode struct

  PoseWithNode<gtsam::Pose3> A, B;

  // Create linearization points
  gtsam::Pose3 poseA(gtsam::Rot3(), gtsam::Point3(0, 0, 0));

  A.pose = poseA;
  A.node = 6;

  B = A.inverse();

  EXPECT(gtsam::assert_equal(B.pose, A.pose.inverse()));

  EXPECT(6 == A.node);
}

/* ************************************************************************* */
TEST(PoseWithNode, Compose) {
  // Test the compose operator for PoseWithNode struct
  PoseWithNode<gtsam::Pose3> A, AB, B, BC, C, CD, D;

  A.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1));  // start
  A.node = 1;

  // First test a translation only
  gtsam::Pose3 poseAB(gtsam::Rot3(), gtsam::Point3(1, 1, 1));
  AB.pose = poseAB;
  AB.node = 1;

  B = A.compose(AB);

  EXPECT(gtsam::assert_equal(A.pose.compose(AB.pose), B.pose));
  EXPECT(2 == B.node);

  // Then rotation only
  gtsam::Pose3 poseBC(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3());
  BC.pose = poseBC;
  BC.node = 1;

  C = B.compose(BC);

  EXPECT(gtsam::assert_equal(B.pose.compose(BC.pose), C.pose));
  EXPECT(3 == C.node);
}

/* ************************************************************************* */
TEST(PoseWithNode, Between) {
  // Test the between operator for the PoseWithNode struct
  PoseWithNode<gtsam::Pose3> A, B, C;

  A.pose = gtsam::Pose3();
  A.node = 1;

  C.pose = gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0));
  C.node = 5;

  B = A.between(C);
  gtsam::Pose3 B_pose =
      gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(1, 0, 0));
  EXPECT(gtsam::assert_equal(B_pose, B.pose));
  EXPECT(4 == B.node);
}

/* ************************************************************************* */
TEST(PoseWithNode, Norm) {
  // Test the between operator for the PoseWithNode struct
  PoseWithNode<gtsam::Pose3> A, B, C;

  A.pose = gtsam::Pose3();
  A.node = 1;

  EXPECT(0 == A.avg_rot_norm());
  EXPECT(0 == A.avg_trans_norm());

  B.pose = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(1, 0, 0));
  B.node = 5;
  EXPECT(0.2 == B.avg_trans_norm());

  C.pose = gtsam::Pose3(gtsam::Rot3(0, 0, 1, 0), gtsam::Point3(0, 0, 0));
  C.node = 5;
  EXPECT_DOUBLES_EQUAL(3.1415927 / 5, C.avg_rot_norm(), 1e-6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
