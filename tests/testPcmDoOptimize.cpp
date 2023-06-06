/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>
#include <vector>

#include <gtsam/sam/RangeFactor.h>
#include "KimeraRPGO/outlier/Pcm.h"

using KimeraRPGO::MultiRobotAlignMethod;
using KimeraRPGO::OutlierRemoval;
using KimeraRPGO::Pcm3D;
using KimeraRPGO::PcmParams;

/* ************************************************************************* */
TEST(PcmDoOptimize, Odometry) {
  // test that when opdemtry edge is received added but return false
  PcmParams params;
  params.lc_threshold = 1.0;
  params.odom_threshold = -1;

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

  EXPECT(nfg.size() == size_t(1));
  EXPECT(est.size() == size_t(1));

  // add odometry
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  bool do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);
  EXPECT(nfg.size() == size_t(2));
  EXPECT(est.size() == size_t(2));
  EXPECT(do_optimize == false);
}

TEST(PcmDoOptimize, OdometryNoPrior) {
  // test that when opdemtry edge is received added but return false
  PcmParams params;
  params.lc_threshold = 1.0;
  params.odom_threshold = -1;

  OutlierRemoval* pcm = new Pcm3D(params);
  // pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  pcm->removeOutliers(gtsam::NonlinearFactorGraph(), init_vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(0));
  EXPECT(est.size() == size_t(1));

  // add odometry
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  bool do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);
  EXPECT(nfg.size() == size_t(1));
  EXPECT(est.size() == size_t(2));
  EXPECT(do_optimize == false);
}
/* ************************************************************************* */
TEST(PcmDoOptimize, LoopClosure) {
  // test that when loop closure edge is received added and return true
  PcmParams params;
  params.lc_threshold = 1.0;
  params.odom_threshold = -1;

  OutlierRemoval* pcm = new Pcm3D(params);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  pcm->removeOutliers(gtsam::NonlinearFactorGraph(), init_vals, &nfg, &est);

  // add odometry
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  pcm->removeOutliers(factors, vals, &nfg, &est);

  // and aother odom
  vals = gtsam::Values();
  factors = gtsam::NonlinearFactorGraph();  // reset
  gtsam::Pose3 odom2 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1, 0, 0));
  vals.insert(2, odom.compose(odom2));
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(1, 2, odom2, noise));
  pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(2));
  EXPECT(est.size() == size_t(3));

  // loop closure
  factors = gtsam::NonlinearFactorGraph();  // reset
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(2, 0, gtsam::Pose3(), noise));
  bool do_optimize = pcm->removeOutliers(factors, gtsam::Values(), &nfg, &est);

  EXPECT(nfg.size() == size_t(3));
  EXPECT(est.size() == size_t(3));

  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
TEST(PcmDoOptimize, landmarks) {
  // test optimize condition for landmarks
  // first observation: do_optimize = false
  // repeated observatio: do_optimize = true
  PcmParams params;
  params.lc_threshold = 10.0;
  params.odom_threshold = -1;
  params.incremental = true;

  std::vector<char> special_symbs{'l', 'u'};  // for landmarks
  OutlierRemoval* pcm =
      new Pcm3D(params, MultiRobotAlignMethod::NONE, 0.0, special_symbs);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  pcm->removeOutliers(gtsam::NonlinearFactorGraph(), init_vals, &nfg, &est);

  // add first landmark observation
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Pose3 meas1 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0));
  gtsam::Key landmark_key = gtsam::Symbol('l', 0);
  vals.insert(landmark_key, meas1);
  factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(0, landmark_key, meas1, noise));
  bool do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(1));
  EXPECT(est.size() == size_t(2));
  EXPECT(do_optimize == false);

  // add odometry
  vals = gtsam::Values();  // reset
  factors = gtsam::NonlinearFactorGraph();
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(2));
  EXPECT(est.size() == size_t(3));
  EXPECT(do_optimize == false);

  // add landmark recurrence
  vals = gtsam::Values();  // reset
  factors = gtsam::NonlinearFactorGraph();
  gtsam::Pose3 meas2 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0));
  factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(1, landmark_key, meas2, noise));
  do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(size_t(3) == nfg.size());
  EXPECT(size_t(3) == est.size());
  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
TEST(PcmDoOptimize, Beacon) {
  // test optimize condition for Beacon
  // first observation: do_optimize = false
  // repeated observatio: do_optimize = true
  PcmParams params;
  params.lc_threshold = 1.0;
  params.odom_threshold = -1;
  params.incremental = true;

  std::vector<char> special_symbs{'l', 'u'};  // for landmarks
  OutlierRemoval* pcm =
      new Pcm3D(params, MultiRobotAlignMethod::NONE, 0.0, special_symbs);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  bool do_optimize =
      pcm->removeOutliers(gtsam::NonlinearFactorGraph(), init_vals, &nfg, &est);

  EXPECT(size_t(0) == nfg.size());
  EXPECT(size_t(1) == est.size());
  EXPECT(do_optimize == false);

  // add first Beacon observation
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors;

  gtsam::Key Beacon_key = gtsam::Symbol('u', 0);

  // Add a PriorFactor for the Beacon
  gtsam::Vector6 prior_precisions;
  prior_precisions.head<3>().setConstant(10.0);
  prior_precisions.tail<3>().setConstant(0.0);
  static const gtsam::SharedNoiseModel& prior_noise =
      gtsam::noiseModel::Diagonal::Precisions(prior_precisions);
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      Beacon_key, gtsam::Pose3(), prior_noise));

  double meas1 = 1.4;
  static const gtsam::SharedNoiseModel& rnoise =
      gtsam::noiseModel::Isotropic::Variance(1, 0.01);

  vals.insert(Beacon_key, meas1);
  factors.add(gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>(
      0, Beacon_key, meas1, rnoise));

  do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(size_t(2) == nfg.size());
  EXPECT(size_t(2) == est.size());
  EXPECT(do_optimize == true);

  // add odometry
  vals = gtsam::Values();  // reset
  factors = gtsam::NonlinearFactorGraph();
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(3));
  EXPECT(est.size() == size_t(3));
  EXPECT(do_optimize == false);

  // add Beacon recurrence
  vals = gtsam::Values();  // reset
  factors = gtsam::NonlinearFactorGraph();
  double meas2 = 1;
  factors.add(gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>(
      1, Beacon_key, meas2, rnoise));
  do_optimize = pcm->removeOutliers(factors, vals, &nfg, &est);

  EXPECT(nfg.size() == size_t(4));
  EXPECT(est.size() == size_t(3));
  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
