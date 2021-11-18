/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
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

/* ************************************************************************* */
TEST(RobustSolver, LandmarkPcm) {
  RobustSolverParams params;
  params.setPcm3DParams(5.0, 2.5, Verbosity::QUIET);
  std::vector<char> special_symbs{'l'};  // for landmarks
  params.specialSymbols = special_symbs;

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
  pgo->update(gtsam::NonlinearFactorGraph(), init_vals);

  // add odometries
  for (size_t i = 0; i < 2; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add more odometries a
  for (size_t i = 2; i < 5; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add a good and bad single robot loop closures
  gtsam::NonlinearFactorGraph lc_factors;
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key a2 = gtsam::Symbol('a', 2);
  gtsam::Key a3 = gtsam::Symbol('a', 3);
  gtsam::Key a4 = gtsam::Symbol('a', 4);
  gtsam::Key a5 = gtsam::Symbol('a', 5);

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a3,
      a2,
      gtsam::Pose3(gtsam::Rot3::Rz(-1.57), gtsam::Point3(0, 0.9, 0)),
      noise));

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4,
      a1,
      gtsam::Pose3(gtsam::Rot3::Rz(3.14), gtsam::Point3(2.1, 1.1, 2.5)),
      noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(6));
  EXPECT(est.size() == size_t(6));

  // Now start adding landmarks
  // Add first observation
  gtsam::Vector6 ldmk_prec;
  ldmk_prec.head<3>().setConstant(0);  // rotation precision
  ldmk_prec.tail<3>().setConstant(25);
  static const gtsam::SharedNoiseModel& lmk_noise =
      gtsam::noiseModel::Diagonal::Precisions(ldmk_prec);

  gtsam::NonlinearFactorGraph landmark_factors;
  gtsam::Values landmark_values;
  gtsam::Key l0 = gtsam::Symbol('l', 0);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a1, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), lmk_noise));
  landmark_values.insert(l0,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(7));
  EXPECT(est.size() == size_t(7));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(7));

  // add a reobservation that's not consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(7));

  // Add another landmark

  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  gtsam::Key l1 = gtsam::Symbol('l', 1);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a2, l1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), lmk_noise));
  landmark_values.insert(l1,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(9));
  EXPECT(est.size() == size_t(8));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5, l1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(10));
  EXPECT(est.size() == size_t(8));
}

/* ************************************************************************* */
TEST(RobustSolver, LandmarkPcmSimple) {
  RobustSolverParams params;
  params.setPcmSimple3DParams(0.3, 0.05, Verbosity::QUIET);
  std::vector<char> special_symbs{'l'};  // for landmarks
  params.specialSymbols = special_symbs;

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
  pgo->update(gtsam::NonlinearFactorGraph(), init_vals);

  // add odometries
  for (size_t i = 0; i < 2; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add more odometries a
  for (size_t i = 2; i < 5; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add a good and bad single robot loop closures
  gtsam::NonlinearFactorGraph lc_factors;
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key a2 = gtsam::Symbol('a', 2);
  gtsam::Key a3 = gtsam::Symbol('a', 3);
  gtsam::Key a4 = gtsam::Symbol('a', 4);
  gtsam::Key a5 = gtsam::Symbol('a', 5);

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a3,
      a2,
      gtsam::Pose3(gtsam::Rot3::Rz(-1.57), gtsam::Point3(0, 0.9, 0)),
      noise));

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4,
      a1,
      gtsam::Pose3(gtsam::Rot3::Rz(3.14), gtsam::Point3(2.1, 1.1, 2.5)),
      noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(6));
  EXPECT(est.size() == size_t(6));

  // Now start adding landmarks
  // Add first observation
  gtsam::Vector6 ldmk_prec;
  ldmk_prec.head<3>().setConstant(0);  // rotation precision
  ldmk_prec.tail<3>().setConstant(25);
  static const gtsam::SharedNoiseModel& lmk_noise =
      gtsam::noiseModel::Diagonal::Precisions(ldmk_prec);

  gtsam::NonlinearFactorGraph landmark_factors;
  gtsam::Values landmark_values;
  gtsam::Key l0 = gtsam::Symbol('l', 0);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a1, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), lmk_noise));
  landmark_values.insert(l0,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(7));
  EXPECT(est.size() == size_t(7));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(7));

  // add a reobservation that's not consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(7));

  // Add another landmark

  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  gtsam::Key l1 = gtsam::Symbol('l', 1);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a2, l1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), lmk_noise));
  landmark_values.insert(l1,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(9));
  EXPECT(est.size() == size_t(8));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5, l1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(10));
  EXPECT(est.size() == size_t(8));
}

/* ************************************************************************* */
TEST(RobustSolver, LandmarkNoReject) {
  RobustSolverParams params;
  params.setNoRejection(Verbosity::QUIET);
  std::vector<char> special_symbs{'l'};  // for landmarks
  params.specialSymbols = special_symbs;

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
  pgo->update(gtsam::NonlinearFactorGraph(), init_vals);

  // add odometries
  for (size_t i = 0; i < 2; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add more odometries a
  for (size_t i = 2; i < 5; i++) {
    gtsam::Values odom_val;
    gtsam::NonlinearFactorGraph odom_factor;
    gtsam::Matrix3 R;
    R.row(0) << 0, -1, 0;
    R.row(1) << 1, 0, 0;
    R.row(2) << 0, 0, 1;
    gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(1, 0, 0));
    static const gtsam::SharedNoiseModel& noiseOdom =
        gtsam::noiseModel::Isotropic::Variance(6, 0.1);
    gtsam::Key key_prev = gtsam::Symbol('a', i);
    gtsam::Key key_new = gtsam::Symbol('a', i + 1);

    odom_val.insert(key_new, odom);

    odom_factor.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_prev, key_new, odom, noiseOdom));
    pgo->update(odom_factor, odom_val);
  }

  // add loop closures
  gtsam::NonlinearFactorGraph lc_factors;
  gtsam::Key a1 = gtsam::Symbol('a', 1);
  gtsam::Key a2 = gtsam::Symbol('a', 2);
  gtsam::Key a3 = gtsam::Symbol('a', 3);
  gtsam::Key a4 = gtsam::Symbol('a', 4);
  gtsam::Key a5 = gtsam::Symbol('a', 5);

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a3,
      a2,
      gtsam::Pose3(gtsam::Rot3::Rz(-1.57), gtsam::Point3(0, 0.9, 0)),
      noise));

  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4,
      a1,
      gtsam::Pose3(gtsam::Rot3::Rz(3.14), gtsam::Point3(2.1, 1.1, 2.5)),
      noise));

  pgo->update(lc_factors, gtsam::Values());

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();

  EXPECT(nfg.size() == size_t(7));
  EXPECT(est.size() == size_t(6));

  // Now start adding landmarks
  // Add first observation
  gtsam::Vector6 ldmk_prec;
  ldmk_prec.head<3>().setConstant(0);  // rotation precision
  ldmk_prec.tail<3>().setConstant(25);
  static const gtsam::SharedNoiseModel& lmk_noise =
      gtsam::noiseModel::Diagonal::Precisions(ldmk_prec);

  gtsam::NonlinearFactorGraph landmark_factors;
  gtsam::Values landmark_values;
  gtsam::Key l0 = gtsam::Symbol('l', 0);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a1, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), lmk_noise));
  landmark_values.insert(l0,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(7));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  gtsam::Matrix3 R;
  R.row(0) << 0, -1, 0;
  R.row(1) << 1, 0, 0;
  R.row(2) << 0, 0, 1;
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5,
      l0,
      gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(0, -1, 0)),
      lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(9));
  EXPECT(est.size() == size_t(7));

  // add a reobservation that's not consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a4, l0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(10));
  EXPECT(est.size() == size_t(7));

  // Add another landmark

  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  gtsam::Key l1 = gtsam::Symbol('l', 1);
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a2, l1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, -1, 0)), lmk_noise));
  landmark_values.insert(l1,
                         gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(11));
  EXPECT(est.size() == size_t(8));

  // add a reobservation should be consistent
  landmark_factors = gtsam::NonlinearFactorGraph();
  landmark_values = gtsam::Values();
  landmark_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      a5, l1, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(2, 0, 0)), lmk_noise));
  pgo->update(landmark_factors, landmark_values);

  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(12));
  EXPECT(est.size() == size_t(8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
