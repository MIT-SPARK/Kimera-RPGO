#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "kimera_rpgo/rpgo.h"
#include "test_config.h"

using namespace kimera_rpgo;

/* ************************************************************************* */
TEST(RpgoGradientDescent2D, GradientDescent) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::GRADIENT;
  config.solver_config.setGradientParamsDefault();
  config.solver_config.optimizer_params->maxIterations = 10000;

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_2d.g2o";
  test.loadG2o(g2o_file, false);
  test.fixFirstPose(false);
  test.run();

  const auto& result = test.getResult();

  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose2(), result.at<gtsam::Pose2>(0), 1.0e-02));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose2(gtsam::Rot2(0.1), gtsam::Point2(9, -0.09)),
      result.at<gtsam::Pose2>(9),
      1.0e-1));
}

/* ************************************************************************* */
TEST(RpgoGradientDescent2D, ConjugateGradientDescent) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::GRADIENT;
  config.solver_config.setGradientParamsDefault();
  config.solver_config.optimizer_params->maxIterations = 10000;

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_2d.g2o";
  test.loadG2o(g2o_file, false);
  test.fixFirstPose(false);
  test.run();

  const auto& result = test.getResult();

  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose2(), result.at<gtsam::Pose2>(0), 1.0e-02));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose2(gtsam::Rot2(0.1), gtsam::Point2(9, -0.09)),
      result.at<gtsam::Pose2>(9),
      1.0e-01));
}

/* ************************************************************************* */
TEST(RpgoGradientDescent3D, GradientDescent) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::GRADIENT;
  config.solver_config.setGradientParamsDefault();
  config.solver_config.optimizer_params->maxIterations = 100000;
  // This usually requires many iterations

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_3d.g2o";
  test.loadG2o(g2o_file, true);
  test.fixFirstPose(true);
  test.run();

  const auto& result = test.getResult();

  gtsam::Key key0 = gtsam::Symbol('d', 0);
  gtsam::Key key9 = gtsam::Symbol('d', 9);
  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(), result.at<gtsam::Pose3>(key0), 1e-02));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(9, 0.09, -0.09)),
      result.at<gtsam::Pose3>(key9),
      1.0e-01));
}

/* ************************************************************************* */
TEST(RpgoGradientDescent3D, ConjugateGradientDescent) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::GRADIENT;
  config.solver_config.setGradientParamsDefault();
  config.solver_config.optimizer_params->maxIterations = 10000;

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_3d.g2o";
  test.loadG2o(g2o_file, true);
  test.fixFirstPose(true);
  test.run();

  const auto& result = test.getResult();

  gtsam::Key key0 = gtsam::Symbol('d', 0);
  gtsam::Key key9 = gtsam::Symbol('d', 9);
  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(), result.at<gtsam::Pose3>(key0), 1.0e-02));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(9, 0.09, -0.09)),
      result.at<gtsam::Pose3>(key9),
      1.0e-01));
}

/* ************************************************************************* */
