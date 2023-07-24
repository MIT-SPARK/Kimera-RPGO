/**
 * @file    testGnc.cpp
 * @brief   Unit test for solver with GNC
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/TypeUtils.h"
#include "test_config.h"

using namespace KimeraRPGO;

/* ************************************************************************* */
TEST(RobustSolver, TemporaryFactors) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  std::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  // High PCM threshold so only use GNC
  params.setPcm3DParams(100.0, 100.0, Verbosity::VERBOSE);
  params.setGncInlierCostThresholdsAtProbability(0.01);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  pgo->update(*nfg, *values);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();
  gtsam::Vector weights = pgo->getGncWeights();
  EXPECT(nfg_out.size() == size_t(52));
  EXPECT(values_out.size() == size_t(50));
  EXPECT(weights.size() == size_t(52));
  // Expect all loop closures to be rejected
  EXPECT(weights.segment(49, 3).sum() == 0);
  EXPECT(weights.sum() == 49);
  EXPECT(pgo->getNumLCInliers() == 0);

  // Add temporary factors
  gtsam::Values temp_values;
  gtsam::NonlinearFactorGraph temp_factors;
  temp_values.insert(gtsam::Symbol('b', 0), gtsam::Pose3());
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);
  temp_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('b', 0), gtsam::Pose3(), noise));

  pgo->updateTempFactorsValues(temp_factors, temp_values);
  pgo->forceUpdate();

  nfg_out = pgo->getFactorsUnsafe();
  gtsam::NonlinearFactorGraph temp_nfg_out = pgo->getTempFactorsUnsafe();
  values_out = pgo->calculateEstimate();
  weights = pgo->getGncWeights();
  EXPECT(nfg_out.size() == size_t(52));
  EXPECT(temp_nfg_out.size() == size_t(1));
  EXPECT(values_out.size() == size_t(50));
  EXPECT(weights.size() == size_t(52));
  EXPECT(pgo->getNumLCInliers() == 0);

  // Remove temporary 
  temp_values = gtsam::Values();
  temp_factors = gtsam::NonlinearFactorGraph();

  pgo->clearTempFactorsValues();
  pgo->forceUpdate();

  nfg_out = pgo->getFactorsUnsafe();
  temp_nfg_out = pgo->getTempFactorsUnsafe();
  values_out = pgo->calculateEstimate();
  weights = pgo->getGncWeights();
  EXPECT(nfg_out.size() == size_t(52));
  EXPECT(temp_nfg_out.size() == size_t(0));
  EXPECT(values_out.size() == size_t(50));
  EXPECT(weights.size() == size_t(52));
  // Expect all loop closures to be rejected
  EXPECT(weights.segment(49, 3).sum() == 0);
  EXPECT(weights.sum() == 49);
  EXPECT(pgo->getNumLCInliers() == 0);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
