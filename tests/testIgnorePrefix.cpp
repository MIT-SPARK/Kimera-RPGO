/**
 * @file    testRemoveLastLoopClosure.cpp
 * @brief   Unit test functionality to remove last loop closure
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <memory>
#include <random>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/TypeUtils.h"
#include "test_config.h"

using namespace KimeraRPGO;

void buildTestGraph(gtsam::NonlinearFactorGraph* factors,
                    gtsam::Values* values) {
  /*
             <-
  a -->-->-->-A
  b -->-->-->-v
             <-          */
  // initialize first (w/o prior)
  gtsam::Key init_key_a = gtsam::Symbol('a', 0);
  values->insert(init_key_a, gtsam::Pose3());
  static const gtsam::SharedNoiseModel& noisePriorA =
      gtsam::noiseModel::Isotropic::Variance(6, 10e-8);
  factors->add(gtsam::PriorFactor<gtsam::Pose3>(
      init_key_a, gtsam::Pose3(), noisePriorA));

  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 init_b = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0));
  values->insert(init_key_b, init_b);
  static const gtsam::SharedNoiseModel& noisePriorB =
      gtsam::noiseModel::Isotropic::Variance(6, 1);
  factors->add(
      gtsam::PriorFactor<gtsam::Pose3>(init_key_b, init_b, noisePriorB));

  // add odometries (3 more)
  for (size_t i = 0; i < 3; i++) {
    gtsam::Values vals;
    gtsam::NonlinearFactorGraph odom_factors;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 10);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    values->insert(key_a_new, odom);
    values->insert(key_b_new, odom);

    factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
    factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
  }

  // add more odometries a
  for (size_t i = 3; i < 5; i++) {
    gtsam::Values odom_val_a;
    gtsam::NonlinearFactorGraph odom_factor_a;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 10);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);

    values->insert(key_a_new, odom);

    factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
  }

  // add more odometries b
  for (size_t i = 3; i < 5; i++) {
    gtsam::Values odom_val_b;
    gtsam::NonlinearFactorGraph odom_factor_b;
    gtsam::Matrix3 R;
    R.row(0) << 0, 1, 0;
    R.row(1) << -1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 10);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    values->insert(key_b_new, odom);

    factors->add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
  }
}

/* ************************************************************************* */
TEST(RobustSolver, IgnoreSinglePrefixA) {
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(100, 100, Verbosity::QUIET);
  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 10e-8);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  buildTestGraph(&nfg, &est);

  pgo->update(nfg, est);

  // Basic check before removing loop closures
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)),
                          est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)),
                          0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 1, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));

  // Add two loop closures
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('a', 0), gtsam::Pose3(), noise));
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 5),
      gtsam::Symbol('a', 3),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(1, 0, 0)),
      noise));
  pgo->update(factors);
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(14));
  EXPECT(est.size() == size_t(12));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(), est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)), 10e-6));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 0, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));

  // Ignore all loop closures with prefix a
  pgo->ignorePrefix('a');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  std::vector<char> ignored_prefixes = pgo->getIgnoredPrefixes();
  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)),
                          est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)),
                          0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 1, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));
  EXPECT(ignored_prefixes.size() == size_t(1));
  EXPECT(ignored_prefixes[0] == 'a');

  // Revive all loop closures with prefix a
  pgo->revivePrefix('a');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  ignored_prefixes = pgo->getIgnoredPrefixes();
  EXPECT(nfg.size() == size_t(14));
  EXPECT(est.size() == size_t(12));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(), est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)), 0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 0, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));
  EXPECT(ignored_prefixes.size() == size_t(0));
}

/* ************************************************************************* */
TEST(RobustSolver, IgnoreSinglePrefixB) {
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(100, 100, Verbosity::QUIET);
  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 10e-8);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  buildTestGraph(&nfg, &est);

  pgo->update(nfg, est);

  // Basic check before removing loop closures
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)),
                          est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)),
                          0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 1, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));

  // Add two loop closures
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('a', 0), gtsam::Pose3(), noise));
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 5),
      gtsam::Symbol('a', 3),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(1, 0, 0)),
      noise));
  pgo->update(factors);
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(14));
  EXPECT(est.size() == size_t(12));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(), est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)), 10e-6));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 0, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));

  // Ignore all loop closures with prefix b
  pgo->ignorePrefix('b');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  std::vector<char> ignored_prefixes = pgo->getIgnoredPrefixes();
  EXPECT(nfg.size() == size_t(13));
  EXPECT(est.size() == size_t(12));
  EXPECT(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)),
                          est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)),
                          0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 0, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));
  EXPECT(ignored_prefixes.size() == size_t(1));
  EXPECT(ignored_prefixes[0] == 'b');

  // Revive all loop closures with prefix b
  pgo->revivePrefix('b');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  ignored_prefixes = pgo->getIgnoredPrefixes();
  EXPECT(nfg.size() == size_t(14));
  EXPECT(est.size() == size_t(12));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(), est.at<gtsam::Pose3>(gtsam::Symbol('b', 0)), 0.01));
  EXPECT(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(4, 0, 0)),
      est.at<gtsam::Pose3>(gtsam::Symbol('a', 5)),
      10e-6));
  EXPECT(ignored_prefixes.size() == size_t(0));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */