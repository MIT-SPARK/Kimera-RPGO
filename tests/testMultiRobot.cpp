/**
 * @file    testMutiRobot.cpp
 * @brief   Unit test for pcm for multirobot uses
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <memory>
#include <random>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/TypeUtils.h"
#include "test_config.h"

using namespace KimeraRPGO;

/* ************************************************************************* */
TEST(RobustSolver, multiRobotPcm) {
  /*
          <--
            A
  a -->-->-->
  b -->-->-->
            V
          <--            */
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(3.0, 0.05, Verbosity::QUIET);
  params.setMultiRobotAlignMethod(MultiRobotAlignMethod::GNC);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Key init_key_a = gtsam::Symbol('a', 0);
  gtsam::Values init_vals;
  init_vals.insert(init_key_a, gtsam::Pose3());

  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 init_b = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0));
  init_vals.insert(init_key_b, init_b);
  pgo->update(gtsam::NonlinearFactorGraph(), init_vals);

  // add odometries (4 more)
  for (size_t i = 0; i < 3; i++) {
    gtsam::Values vals;
    gtsam::NonlinearFactorGraph odom_factors;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    vals.insert(key_a_new, odom);
    vals.insert(key_b_new, odom);

    odom_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
    odom_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
    pgo->update(odom_factors, vals);
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
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);

    odom_val_a.insert(key_a_new, odom);

    odom_factor_a.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
    pgo->update(odom_factor_a, odom_val_a);
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
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    odom_val_b.insert(key_b_new, odom);

    odom_factor_b.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
    pgo->update(odom_factor_b, odom_val_b);
  }

  // loop closures
  gtsam::NonlinearFactorGraph lc_factors;

  // first a good loop closure
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key b1 = gtsam::Symbol('b', 1);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b1, a1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), noise));

  gtsam::Key a4 = gtsam::Symbol('a', 4);
  gtsam::Key b4 = gtsam::Symbol('b', 4);
  gtsam::Matrix3 R;
  R.row(0) << -1, 0, 0;
  R.row(1) << 0, -1, 0;
  R.row(2) << 0, 0, 1;
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b4, a4, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(-1, 0, 0)), noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));

  // add bad loop closures
  lc_factors = gtsam::NonlinearFactorGraph();
  gtsam::Key a2 = gtsam::Symbol('a', 2);
  gtsam::Key b2 = gtsam::Symbol('b', 2);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a2, b2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), noise));

  gtsam::Key a5 = gtsam::Symbol('a', 5);
  gtsam::Key b5 = gtsam::Symbol('b', 5);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b5, a5, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -3.3, 0)), noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));

  // add a good and bad single robot loop closures
  lc_factors = gtsam::NonlinearFactorGraph();
  gtsam::Key a3 = gtsam::Symbol('a', 3);

  // good
  R.row(0) << 0, 1, 0;
  R.row(1) << -1, 0, 0;
  R.row(2) << 0, 0, 1;
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4, a1, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(0, 3, 0)), noise));
  // bad lc
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4,
      a2,
      gtsam::Pose3(gtsam::Rot3::Rz(-1.54), gtsam::Point3(0, 2, 0)),
      noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(13));
  EXPECT(est.size() == size_t(12));
}

/* ************************************************************************* */
TEST(RobustSolver, multiRobotPcmSimple) {
  /*
          <--
            A
  a -->-->-->
  b -->-->-->
            V
          <--            */
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(0.04, 0.01, Verbosity::QUIET);
  params.setMultiRobotAlignMethod(MultiRobotAlignMethod::GNC);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.1);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  // initialize first (w/o prior)
  gtsam::Key init_key_a = gtsam::Symbol('a', 0);
  gtsam::Values init_vals;
  init_vals.insert(init_key_a, gtsam::Pose3());

  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 init_b = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0));
  init_vals.insert(init_key_b, init_b);
  pgo->update(gtsam::NonlinearFactorGraph(), init_vals);

  // add odometries (4 more)
  for (size_t i = 0; i < 3; i++) {
    gtsam::Values vals;
    gtsam::NonlinearFactorGraph odom_factors;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    vals.insert(key_a_new, odom);
    vals.insert(key_b_new, odom);

    odom_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
    odom_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
    pgo->update(odom_factors, vals);
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
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_a_prev = gtsam::Symbol('a', i);
    gtsam::Key key_a_new = gtsam::Symbol('a', i + 1);

    odom_val_a.insert(key_a_new, odom);

    odom_factor_a.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_a_prev, key_a_new, odom, noiseOdom));
    pgo->update(odom_factor_a, odom_val_a);
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
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_b_prev = gtsam::Symbol('b', i);
    gtsam::Key key_b_new = gtsam::Symbol('b', i + 1);

    odom_val_b.insert(key_b_new, odom);

    odom_factor_b.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_b_prev, key_b_new, odom, noiseOdom));
    pgo->update(odom_factor_b, odom_val_b);
  }

  // loop closures
  gtsam::NonlinearFactorGraph lc_factors;

  // first a good loop closure
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key b1 = gtsam::Symbol('b', 1);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b1, a1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), noise));

  gtsam::Key a4 = gtsam::Symbol('a', 4);
  gtsam::Key b4 = gtsam::Symbol('b', 4);
  gtsam::Matrix3 R;
  R.row(0) << -1, 0, 0;
  R.row(1) << 0, -1, 0;
  R.row(2) << 0, 0, 1;
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b4, a4, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(-1, 0, 0)), noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));

  // add bad loop closures
  lc_factors = gtsam::NonlinearFactorGraph();
  gtsam::Key a2 = gtsam::Symbol('a', 2);
  gtsam::Key b2 = gtsam::Symbol('b', 2);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a2, b2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), noise));

  gtsam::Key a5 = gtsam::Symbol('a', 5);
  gtsam::Key b5 = gtsam::Symbol('b', 5);
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      b5, a5, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -3.3, 0)), noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(12));

  // add a good and bad single robot loop closures
  lc_factors = gtsam::NonlinearFactorGraph();
  gtsam::Key a3 = gtsam::Symbol('a', 3);

  // good
  R.row(0) << 0, 1, 0;
  R.row(1) << -1, 0, 0;
  R.row(2) << 0, 0, 1;
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4, a1, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(0, 3, 0)), noise));
  // bad lc
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4,
      a2,
      gtsam::Pose3(gtsam::Rot3::Rz(-1.54), gtsam::Point3(0, 2, 0)),
      noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(13));
  EXPECT(est.size() == size_t(12));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
