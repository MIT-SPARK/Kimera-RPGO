/**
 * @file    testPoseWithCovariance.cpp
 * @brief   Unit test for PoseWithCovariance and calculations 
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include <gtsam/slam/dataset.h>

#include "RobustPGO/graph_utils/graph_utils.h" 

#include "test_config.h"

/* ************************************************************************* */
TEST(PoseWithCovariance, Inverse)
{
  gtsam::GraphAndValues gv_unordered = gtsam::load3D(std::string(DATASET_PATH) + "/unordered.g2o");
  gtsam::NonlinearFactorGraph nfg1 = *gv_unordered.first;

  graph_utils::orderGraph<gtsam::Pose3>(nfg1);

  gtsam::GraphAndValues gv_ordered = gtsam::load3D(std::string(DATASET_PATH) + "/ordered.g2o");
  gtsam::NonlinearFactorGraph nfg2 = *gv_ordered.first;

  EXPECT(gtsam::assert_equal(nfg1, nfg2));
}


/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
