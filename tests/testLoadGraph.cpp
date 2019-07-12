/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "RobustPGO/RobustPGO.h"
#include "RobustPGO/pcm/pcm.h" 
#include "test_config.h"

/* ************************************************************************* */
TEST(RobustPGO, Load1)
{
  // load graph
  // read g2o file for robot a 
  gtsam::GraphAndValues gv = gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // set up RobustPGO solver 
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(0.0, 10.0); // set odom check to be small
  pcm->setQuiet(); // turn off print messages for pcm

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->setQuiet(); // turn off print messages

  // Create prior
  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(init_key, values.at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(nfg, values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges + prior (no lc should have passed)
  EXPECT(gtsam::assert_equal(nfg_out.size(), size_t(50)));
  EXPECT(gtsam::assert_equal(values_out.size(), size_t(50)));
}

/* ************************************************************************* */
TEST(RobustPGO, Add1)
{
  // load graph for robot a (same as above)
  gtsam::GraphAndValues gv = gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(0.0, 10.0); // set odom check to be small
  pcm->setQuiet(); // turn off print messages for pcm

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->setQuiet(); // turn off print messages

  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(init_key, values.at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(nfg, values, init);// first load 

  // add graph 
  // read g2o file for robot b
  gtsam::GraphAndValues gv_b = gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");
  gtsam::NonlinearFactorGraph nfg_b = *gv_b.first;
  gtsam::Values values_b = *gv_b.second;

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values.at<gtsam::Pose3>(init_key).between(values_b.at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(init_key, init_key_b, transform_ab, noise);

  // add graph 
  pgo->addGraph(nfg_b, values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges + prior + between (no lc should have passed)
  EXPECT(gtsam::assert_equal(nfg_out.size(), size_t(92)));
  EXPECT(gtsam::assert_equal(values_out.size(), size_t(92)));
}

/* ************************************************************************* */
TEST(RobustPGO, Load1NoPrior)
{
  // load graph
  // read g2o file for robot a 
  gtsam::GraphAndValues gv = gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // set up RobustPGO solver 
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(0.0, 10.0); // set odom check to be small
  pcm->setQuiet(); // turn off print messages for pcm

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->setQuiet(); // turn off print messages

  gtsam::Key init_key = gtsam::Symbol('a', 0);

  // Load graph using prior
  pgo->loadGraph<gtsam::Pose3>(nfg, values, init_key);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since odom check threshold is 0, should only have the odom edges
  EXPECT(gtsam::assert_equal(nfg_out.size(), size_t(49)));
  EXPECT(gtsam::assert_equal(values_out.size(), size_t(50)));
}
/* ************************************************************************* */
TEST(RobustPGO, Load2)
{
  // load graph
  // read g2o file for robot a 
  gtsam::GraphAndValues gv = gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  // set up RobustPGO solver 
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(100.0, 100.0); // set thresholds to be large
  pcm->setQuiet(); // turn off print messages for pcm

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->setQuiet(); // turn off print messages

  // Create prior
  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(init_key, values.at<gtsam::Pose3>(init_key), noise);

  // Load graph using prior
  pgo->loadGraph(nfg, values, init);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(gtsam::assert_equal(nfg_out.size(), size_t(53)));
  EXPECT(gtsam::assert_equal(values_out.size(), size_t(50)));
}

/* ************************************************************************* */
TEST(RobustPGO, Add2)
{
  // load graph for robot a (same as above)
  gtsam::GraphAndValues gv = gtsam::load3D(std::string(DATASET_PATH) + "/robot_a.g2o");
  gtsam::NonlinearFactorGraph nfg = *gv.first;
  gtsam::Values values = *gv.second;

  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(100.0, 100.0); // set thresholds to be large
  // pcm->setQuiet(); // turn off print messages for pcm

  std::unique_ptr<RobustPGO> pgo;
  pgo.reset(new RobustPGO(pcm));
  pgo->setQuiet(); // turn off print messages

  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::Key init_key = gtsam::Symbol('a', 0);
  gtsam::PriorFactor<gtsam::Pose3> init(init_key, values.at<gtsam::Pose3>(init_key), noise);
  pgo->loadGraph(nfg, values, init);// first load 

  // add graph 
  // read g2o file for robot b
  gtsam::GraphAndValues gv_b = gtsam::load3D(std::string(DATASET_PATH) + "/robot_b.g2o");
  gtsam::NonlinearFactorGraph nfg_b = *gv_b.first;
  gtsam::Values values_b = *gv_b.second;

  // create the between factor for connection
  gtsam::Key init_key_b = gtsam::Symbol('b', 0);
  gtsam::Pose3 transform_ab = values.at<gtsam::Pose3>(init_key).between(values_b.at<gtsam::Pose3>(init_key_b));
  gtsam::BetweenFactor<gtsam::Pose3> bridge(init_key, init_key_b, transform_ab, noise);

  // add graph 
  pgo->addGraph(nfg_b, values_b, bridge);

  gtsam::NonlinearFactorGraph nfg_out = pgo->getFactorsUnsafe();
  gtsam::Values values_out = pgo->calculateEstimate();

  // Since thresholds are high, should have all the edges
  EXPECT(gtsam::assert_equal(nfg_out.size(), size_t(97)));
  EXPECT(gtsam::assert_equal(values_out.size(), size_t(92)));
}
/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
