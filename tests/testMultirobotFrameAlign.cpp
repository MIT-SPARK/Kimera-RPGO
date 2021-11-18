/**
 * @file    testMultirobotFrameAlign.cpp
 * @brief   Unit test for pcm
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/outlier/Pcm.h"

using KimeraRPGO::OutlierRemoval;
using KimeraRPGO::Pcm3D;
using KimeraRPGO::PcmParams;

/* ************************************************************************* */
TEST(Pcm, MultirobotFrameAlign) {
  PcmParams params;
  params.odom_trans_threshold = -1;
  params.odom_rot_threshold = -1;
  params.dist_trans_threshold = 0.3;
  params.dist_rot_threshold = 100.0;

  OutlierRemoval* pcm =
      new Pcm3D(params, KimeraRPGO::MultiRobotAlignMethod::GNC);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // Add first robot
  est.insert(gtsam::Symbol('a', 0), gtsam::Pose3());
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  est.insert(gtsam::Symbol('a', 1), gtsam::Pose3());

  // Add second robot
  est.insert(gtsam::Symbol('b', 0),
             gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0),
      gtsam::Symbol('b', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));
  est.insert(gtsam::Symbol('b', 1),
             gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0)));

  // Add third robot
  est.insert(gtsam::Symbol('c', 0),
             gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(0, 0, 0)));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('c', 0),
      gtsam::Symbol('c', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));
  est.insert(gtsam::Symbol('c', 1),
             gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(-1, 0, 0)));

  // Add loop closures
  //// robot 1-2
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('b', 0), gtsam::Pose3(), noise));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Symbol('b', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));
  //// robot 1-3
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('c', 0), gtsam::Pose3(), noise));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Symbol('c', 1),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
      noise));

  gtsam::NonlinearFactorGraph output_nfg;
  gtsam::Values output_vals;
  pcm->removeOutliers(nfg, est, &output_nfg, &output_vals);

  // Check the updated initial values
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('a', 0)),
                          1e-6));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('a', 1)),
                          1e-6));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('b', 0)),
                          1e-6));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('b', 1)),
                          1e-6));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('c', 0)),
                          1e-6));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          output_vals.at<gtsam::Pose3>(gtsam::Symbol('c', 1)),
                          1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
