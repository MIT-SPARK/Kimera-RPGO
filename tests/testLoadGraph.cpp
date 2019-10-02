/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
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
#include "KimeraRPGO/utils/type_utils.h"
#include "test_config.h"

using namespace KimeraRPGO;

/* ************************************************************************* */
TEST(RobustSolver, Load1) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(0.0, 10.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(*nfg, *values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges + prior
  // (no lc should have passed)
  EXPECT(nfg_out.size() == size_t(50));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, Add1) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(0.0, 10.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(*nfg, *values, init);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  boost::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values->at<gtsam::Pose3>(init_key).between(
      values_b->at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(
      init_key, init_key_b, transform_ab, noise);

  // add graph
  pgo->addGraph(*nfg_b, *values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges + prior +
  // between (no lc should have passed)
  EXPECT(nfg_out.size() == size_t(92));
  EXPECT(values_out.size() == size_t(92));

  // Try add another loop closuer
  // create the between factor for connection
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::Pose3 a1b1 = gtsam::Pose3();
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(key_a1, key_b1, a1b1, noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  EXPECT(nfg_out.size() == size_t(92));
  EXPECT(values_out.size() == size_t(92));
}

/* ************************************************************************* */
TEST(RobustSolver, Load2) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);
  std::vector<char> special_symbs{'l', 'u'};  // for landmarks
  params.specialSymbols = special_symbs;

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(*nfg, *values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(53));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, Add2) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(100.0, 100.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(*nfg, *values, init);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  boost::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values->at<gtsam::Pose3>(init_key).between(
      values_b->at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(
      init_key, init_key_b, transform_ab, noise);

  // add graph
  pgo->addGraph(*nfg_b, *values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(97));
  EXPECT(values_out.size() == size_t(92));

  // Try add another loop closuer
  // create the between factor for connection
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::Pose3 a1b1 = gtsam::Pose3();
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(key_a1, key_b1, a1b1, noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();

  EXPECT(nfg_out.size() == size_t(98));
  EXPECT(values_out.size() == size_t(92));
}

/* ************************************************************************* */
TEST(RobustSolver, Load1NoPrior) {
  // load graph
  // read g2o file for robot a
  gtsam::GraphAndValues gv =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcm3DParams(0.0, 10.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  gtsam::Key init_key = gtsam::Symbol('a', 0);

  // Load graph using prior
  pgo->loadGraph<gtsam::Pose3>(nfg, values, init_key);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges
  EXPECT(nfg_out.size() == size_t(49));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, NoRejectLoad) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setNoRejection(Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(*nfg, *values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(53));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, NoRejectAdd) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setNoRejection(Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(*nfg, *values, init);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  boost::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values->at<gtsam::Pose3>(init_key).between(
      values_b->at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(
      init_key, init_key_b, transform_ab, noise);

  // add graph
  pgo->addGraph(*nfg_b, *values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(97));
  EXPECT(values_out.size() == size_t(92));

  // Try add another loop closuer
  // create the between factor for connection
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::Pose3 a1b1 = gtsam::Pose3();
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(key_a1, key_b1, a1b1, noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();

  EXPECT(nfg_out.size() == size_t(98));
  EXPECT(values_out.size() == size_t(92));
}

/* ************************************************************************* */
TEST(RobustSolver, Load1PcmSimple) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(0.001, 0.0001, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(*nfg, *values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // threshold very low, should only have the odom edges + prior (no lc should
  // have passed)
  EXPECT(nfg_out.size() == size_t(50));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, Add1PcmSimple) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(0.001, 0.0001, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(*nfg, *values, init);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  boost::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values->at<gtsam::Pose3>(init_key).between(
      values_b->at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(
      init_key, init_key_b, transform_ab, noise);

  // add graph
  pgo->addGraph(*nfg_b, *values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Thresholds are close to 0, should only have the odom edges + prior +
  // between (no lc should have passed)
  EXPECT(nfg_out.size() == size_t(92));
  EXPECT(values_out.size() == size_t(92));

  // Try add another loop closuer
  // create the between factor for connection
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::Pose3 a1b1 = gtsam::Pose3();
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(key_a1, key_b1, a1b1, noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();

  EXPECT(nfg_out.size() == size_t(92));
  EXPECT(values_out.size() == size_t(92));
}

/* ************************************************************************* */
TEST(RobustSolver, Load2PcmSimple) {
  // load graph
  // read g2o file for robot a
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(100.0, 100.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  // Create prior
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(*nfg, *values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(53));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
TEST(RobustSolver, Add2PcmSimple) {
  // load graph for robot a (same as above)
  gtsam::NonlinearFactorGraph::shared_ptr nfg;
  gtsam::Values::shared_ptr values;
  boost::tie(nfg, values) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(100.0, 100.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(
      init_key, values->at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(*nfg, *values, init);  // first load

  // add graph
  // read g2o file for robot b
  gtsam::NonlinearFactorGraph::shared_ptr nfg_b;
  gtsam::Values::shared_ptr values_b;
  boost::tie(nfg_b, values_b) =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values->at<gtsam::Pose3>(init_key).between(
      values_b->at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(
      init_key, init_key_b, transform_ab, noise);

  // add graph
  pgo->addGraph(*nfg_b, *values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(nfg_out.size() == size_t(97));
  EXPECT(values_out.size() == size_t(92));

  // Try add another loop closuer
  // create the between factor for connection
  gtsam::Key key_b1 = gtsam::Symbol('b', 1);
  gtsam::Key key_a1 = gtsam::Symbol('a', 1);
  gtsam::Pose3 a1b1 = gtsam::Pose3();
  gtsam::BetweenFactor<gtsam::Pose3> a1tob1(key_a1, key_b1, a1b1, noise);

  gtsam::NonlinearFactorGraph newfactors;
  newfactors.add(a1tob1);
  gtsam::Values newvalues;
  pgo->update(newfactors, newvalues);

  nfg_out = pgo->getFactorsUnsafe();
  values_out = pgo->calculateEstimate();

  EXPECT(nfg_out.size() == size_t(98));
  EXPECT(values_out.size() == size_t(92));
}

/* ************************************************************************* */
TEST(RobustSolver, Load1NoPriorPcmSimple) {
  // load graph
  // read g2o file for robot a
  gtsam::GraphAndValues gv =
      gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // set up KimeraRPGO solver
  RobustSolverParams params;
  params.setPcmSimple3DParams(100.0, 100.0, Verbosity::QUIET);

  std::unique_ptr<RobustSolver> pgo =
      KimeraRPGO::make_unique<RobustSolver>(params);

  gtsam::Key init_key = gtsam::Symbol('a', 0);

  // Load graph using prior
  pgo->loadGraph<gtsam::Pose3>(nfg, values, init_key);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // High threshold, all pass
  EXPECT(nfg_out.size() == size_t(52));
  EXPECT(values_out.size() == size_t(50));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
