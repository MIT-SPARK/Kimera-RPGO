/**
 * @file    testRemovePriorFactors.cpp
 * @brief   Unit test functionality to remove prior factors
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

using KimeraRPGO::RobustSolver;
using KimeraRPGO::RobustSolverParams;
using KimeraRPGO::Verbosity;

/* ************************************************************************* */
TEST(RobustSolver, RemovePriorNoOR) {
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setNoRejection(Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 10e-8);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;

  // Add some between factors
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('a', 0), gtsam::Pose3());
  values.insert(gtsam::Symbol('a', 1), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('b', 1), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('b', 0), gtsam::Pose3());
  values.insert(gtsam::Symbol('b', 1), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1), gtsam::Symbol('a', 2), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('a', 2), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 2), gtsam::Pose3(), noise));

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));

  // Add prior factors
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Pose3(), noise));
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Pose3(), noise));

  pgo->update(factors, values);

  // Basic check before removing loop closures
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(5));

  // Remove all priors with prefix 'a'
  pgo->removePriorFactorsWithPrefix('a');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(6));
  EXPECT(est.size() == size_t(5));

  // Remove all priors with prefix 'b'
  pgo->removePriorFactorsWithPrefix('b');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(5));
  EXPECT(est.size() == size_t(5));
}

/* ************************************************************************* */
TEST(RobustSolver, RemovePriorPcm) {
  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(100, 100, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 10e-8);

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values est;

  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;

  // Add some between factors
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('a', 0), gtsam::Pose3());
  values.insert(gtsam::Symbol('a', 1), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('b', 1), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('b', 0), gtsam::Pose3());
  values.insert(gtsam::Symbol('b', 1), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1), gtsam::Symbol('a', 2), gtsam::Pose3(), noise));
  values.insert(gtsam::Symbol('a', 2), gtsam::Pose3());

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 2), gtsam::Pose3(), noise));

  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));

  // Add prior factors
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Pose3(), noise));
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('b', 0), gtsam::Pose3(), noise));

  pgo->update(factors, values);

  // Basic check before removing loop closures
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(8));
  EXPECT(est.size() == size_t(5));

  // Remove all priors with prefix 'a'
  pgo->removePriorFactorsWithPrefix('a');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(6));
  EXPECT(est.size() == size_t(5));

  // Remove all priors with prefix 'b'
  pgo->removePriorFactorsWithPrefix('b');
  nfg = pgo->getFactorsUnsafe();
  est = pgo->calculateEstimate();
  EXPECT(nfg.size() == size_t(5));
  EXPECT(est.size() == size_t(5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
