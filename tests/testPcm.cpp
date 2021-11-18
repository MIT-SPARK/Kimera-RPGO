/**
 * @file    testPcm.cpp
 * @brief   Unit test for pcm
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "KimeraRPGO/outlier/Pcm.h"

using KimeraRPGO::OutlierRemoval;
using KimeraRPGO::Pcm3D;
using KimeraRPGO::PcmParams;

/* ************************************************************************* */
TEST(Pcm, OdometryCheck) {
  // Here want to test carefully pcm
  // first test odometry check so set pcm thres high
  PcmParams params;
  params.lc_threshold = -1;
  params.odom_threshold = 0.3;

  OutlierRemoval* pcm = new Pcm3D(params);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/ prior)
  gtsam::Values init_vals;
  gtsam::NonlinearFactorGraph init_factors;
  init_vals.insert(0, gtsam::Pose3());
  init_factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), noise));
  pcm->removeOutliers(init_factors, init_vals, &nfg, &est);

  // add odometries
  for (size_t i = 0; i < 3; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    odom_val.insert(i + 1, odom);
    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, odom, noiseOdom));
    pcm->removeOutliers(odom_factor, odom_val, &nfg, &est);
  }

  // Check everything is normal here
  EXPECT(size_t(4) == nfg.size());
  EXPECT(size_t(4) == est.size());

  // Then add loop closure 1 with mahalanobis dist of around 0.29
  gtsam::NonlinearFactorGraph lc_factor1;
  gtsam::Rot3 R_lc1 = gtsam::Rot3::Rz(1.51);
  gtsam::Pose3 lc1 = gtsam::Pose3(R_lc1, gtsam::Point3(0.8, 0, 0));
  static const gtsam::SharedNoiseModel& noiseLc1 =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);
  lc_factor1.add(gtsam::BetweenFactor<gtsam::Pose3>(3, 0, lc1, noiseLc1));

  bool do_optimize =
      pcm->removeOutliers(lc_factor1, gtsam::Values(), &nfg, &est);
  EXPECT(size_t(5) == nfg.size());
  EXPECT(size_t(4) == est.size());
  EXPECT(do_optimize == true);

  // Then add loop closure 2 with mahalanobis dist of around > 0.309
  gtsam::NonlinearFactorGraph lc_factor2;
  gtsam::Rot3 R_lc2 = gtsam::Rot3::Rz(1.51);
  gtsam::Pose3 lc2 = gtsam::Pose3(R_lc2, gtsam::Point3(0.8, 0, 0));
  static const gtsam::SharedNoiseModel& noiseLc2 =
      gtsam::noiseModel::Isotropic::Variance(6, 0.05);
  lc_factor2.add(gtsam::BetweenFactor<gtsam::Pose3>(3, 0, lc2, noiseLc2));

  do_optimize = pcm->removeOutliers(lc_factor2, gtsam::Values(), &nfg, &est);
  EXPECT(size_t(5) == nfg.size());
  EXPECT(size_t(4) == est.size());
  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
TEST(Pcm, ConsistencyCheck) {
  // Here want to test carefully pcm
  // test pcm check so set odom thres high
  PcmParams params;
  params.lc_threshold = 0.5;
  params.odom_threshold = -1;

  OutlierRemoval* pcm = new Pcm3D(params);
  // pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/ prior)
  gtsam::Values init_vals;
  gtsam::NonlinearFactorGraph init_factors;
  init_vals.insert(0, gtsam::Pose3());
  init_factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), noise));
  pcm->removeOutliers(init_factors, init_vals, &nfg, &est);

  // add odometries
  for (size_t i = 0; i < 2; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    odom_val.insert(i + 1, odom);
    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, odom, noiseOdom));
    pcm->removeOutliers(odom_factor, odom_val, &nfg, &est);
  }

  // add odometries (4 more)
  for (size_t i = 2; i < 6; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    odom_val.insert(i + 1, odom);
    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, odom, noiseOdom));
    pcm->removeOutliers(odom_factor, odom_val, &nfg, &est);
  }

  // Check everything is normal here
  EXPECT(size_t(7) == nfg.size());
  EXPECT(size_t(7) == est.size());

  // Then add 2 loop closures (others will be checked with this )
  gtsam::NonlinearFactorGraph lc_factor1;
  gtsam::Rot3 R_lc1 = gtsam::Rot3::Rz(3.1416);
  gtsam::Pose3 lc1 = gtsam::Pose3(R_lc1, gtsam::Point3(0, 0.9, 0));
  static const gtsam::SharedNoiseModel& noiseLc =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);
  lc_factor1.add(gtsam::BetweenFactor<gtsam::Pose3>(3, 0, lc1, noiseLc));
  pcm->removeOutliers(lc_factor1, gtsam::Values(), &nfg, &est);

  // Then add 2 loop closures (others will be checked with this )
  gtsam::NonlinearFactorGraph lc_factor2;
  gtsam::Rot3 R_lc2 = gtsam::Rot3::Rz(3.1416);
  gtsam::Pose3 lc2 = gtsam::Pose3(R_lc2, gtsam::Point3(-1, 0.8, 0));
  lc_factor2.add(gtsam::BetweenFactor<gtsam::Pose3>(4, 0, lc2, noiseLc));
  bool do_optimize =
      pcm->removeOutliers(lc_factor2, gtsam::Values(), &nfg, &est);

  // Check that the two previous loop closures are consistent
  EXPECT(size_t(9) == nfg.size());
  EXPECT(size_t(7) == est.size());
  EXPECT(do_optimize == true);

  // Now add another consistent loop closure
  gtsam::NonlinearFactorGraph lc_factor3;
  gtsam::Rot3 R_lc3 = gtsam::Rot3::Rz(0.99 * 3.1416);
  gtsam::Pose3 lc3 = gtsam::Pose3(R_lc3, gtsam::Point3(-1.8, 0.8, 0));
  lc_factor3.add(gtsam::BetweenFactor<gtsam::Pose3>(5, 0, lc3, noiseLc));
  do_optimize = pcm->removeOutliers(lc_factor3, gtsam::Values(), &nfg, &est);

  // Distances to all two prev lc should be < 0.15
  EXPECT(size_t(10) == nfg.size());
  EXPECT(size_t(7) == est.size());
  EXPECT(do_optimize == true);

  // Now add an inconsistent loop closure
  gtsam::NonlinearFactorGraph lc_factor4;
  gtsam::Rot3 R_lc4 = gtsam::Rot3::Rz(0.98 * 3.1416);
  gtsam::Pose3 lc4 = gtsam::Pose3(R_lc4, gtsam::Point3(-2.6, 0.6, 0));
  lc_factor4.add(gtsam::BetweenFactor<gtsam::Pose3>(6, 0, lc4, noiseLc));
  do_optimize = pcm->removeOutliers(lc_factor4, gtsam::Values(), &nfg, &est);

  // Should only be consistent with lc3, won't make it into max clique
  EXPECT(size_t(10) == nfg.size());
  EXPECT(size_t(7) == est.size());
  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
TEST(Pcm, OdometryCheck2D) {
  // TODO(Yun)
}

/* ************************************************************************* */
TEST(Pcm, ConsistencyCheck2D) {
  // TODO(Yun)
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
