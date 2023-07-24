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
TEST(RobustSolver, GncDefault) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  std::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  // High PCM threshold so only use GNC
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);
  params.setGncInlierCostThresholdsAtProbability(0.01);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init_factor(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  nfg->add(init_factor);
  pgo->update(*nfg, *values);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();
  gtsam::Vector weights = pgo->getGncWeights();
  EXPECT(nfg_out.size() == size_t(53));
  EXPECT(values_out.size() == size_t(50));
  EXPECT(weights.size() == size_t(53));
  // Expect all loop closures to be rejected
  EXPECT(weights.segment(50, 3).sum() == 0);
  EXPECT(weights.sum() == 50);
  EXPECT(pgo->getNumLCInliers() == 0);
}

/* ************************************************************************* */
TEST(RobustSolver, GncHighThreshold) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  std::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  // High PCM threshold so only use GNC
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);
  params.setGncInlierCostThresholdsAtProbability(0.99);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init_factor(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  nfg->add(init_factor);
  pgo->update(*nfg, *values);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();
  gtsam::Vector weights = pgo->getGncWeights();
  EXPECT(nfg_out.size() == size_t(53));
  EXPECT(values_out.size() == size_t(50));
  EXPECT(weights.size() == size_t(53));
  // Expect all loop closures to be accepted
  EXPECT(weights.segment(50, 3).sum() == 3);
  EXPECT(weights.sum() == 53);
  EXPECT(pgo->getNumLCInliers() == 3);
}

/* ************************************************************************* */
TEST(RobustSolver, GncMultirobotDefault) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  std::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  // High PCM threshold so only use GNC
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);
  params.setGncInlierCostThresholds(1.0);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  nfg->add(init);
  pgo->update(*nfg, *values);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  std::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // add robot b
  pgo->update(*nfg_b, *values_b);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();
  gtsam::Vector weights = pgo->getGncWeights();

  EXPECT(nfg_out.size() == size_t(96));
  EXPECT(values_out.size() == size_t(92));
  EXPECT(weights.size() == size_t(96));
  // Expect all loop closures to be rejected
  EXPECT(weights.segment(91, 5).sum() == 0);
  EXPECT(weights.sum() == 91);
  EXPECT(pgo->getNumLCInliers() == 0);

  // Add interrobot loop closures (sould be inliers)
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(
      key_a1, key_b1, gtsam::Pose3(), noise);

  gtsam::Key key_b2 = gtsam::Symbol('b', 2);
  gtsam::Key key_a2 = gtsam::Symbol('a', 2);
  gtsam::BetweenFactor<gtsam::Pose3> a2tob2(
      key_a2, key_b2, gtsam::Pose3(), noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  newfactors.add(a2tob2);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();
  weights = pgo->getGncWeights();

  EXPECT(nfg_out.size() == size_t(98));
  EXPECT(values_out.size() == size_t(92));
  EXPECT(weights.size() == size_t(98));
  // Expect new loop closures to be accepted
  EXPECT(weights.segment(91, 7).sum() == 2);
  EXPECT(weights.sum() == 93);
  EXPECT(pgo->getNumLCInliers() == 2);
}

/* ************************************************************************* */
TEST(RobustSolver, GncMultirobotHighThreshold) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  std::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  // High PCM threshold so only use GNC
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);
  params.setGncInlierCostThresholds(100.0);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  nfg->add(init);
  pgo->update(*nfg, *values);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  std::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // add robot b
  pgo->update(*nfg_b, *values_b);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();
  gtsam::Vector weights = pgo->getGncWeights();

  EXPECT(nfg_out.size() == size_t(96));
  EXPECT(values_out.size() == size_t(92));
  EXPECT(weights.size() == size_t(96));
  // Expect all loop closures to be accepted
  EXPECT(weights.segment(91, 5).sum() == 5);
  EXPECT(weights.sum() == 96);
  EXPECT(pgo->getNumLCInliers() == 5);

  // Add interrobot loop closures (sould be inliers)
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(
      key_a1, key_b1, gtsam::Pose3(), noise);

  gtsam::Key key_b2 = gtsam::Symbol('b', 2);
  gtsam::Key key_a2 = gtsam::Symbol('a', 2);
  gtsam::BetweenFactor<gtsam::Pose3> a2tob2(
      key_a2, key_b2, gtsam::Pose3(), noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  newfactors.add(a2tob2);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();
  weights = pgo->getGncWeights();

  EXPECT(nfg_out.size() == size_t(98));
  EXPECT(values_out.size() == size_t(92));
  EXPECT(weights.size() == size_t(98));
  // Expect all loop closures to be accepted
  EXPECT(weights.segment(91, 7).sum() == 7);
  EXPECT(weights.sum() == 98);
  EXPECT(pgo->getNumLCInliers() == 7);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
