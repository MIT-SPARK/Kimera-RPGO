#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "kimera_rpgo/rpgo.h"
#include "test_config.h"

using namespace kimera_rpgo;

/* ************************************************************************* */
TEST(RpgoGnc2D, GaussNewton) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::LEAST_SQUARES;
  auto gn_params = std::make_shared<GaussNewtonParams>();
  config.solver_config.setLeastSquaresParams(gn_params);
  config.solver_config.setGncParamsDefault();

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_2d_robust.g2o";
  test.loadG2o(g2o_file, false);
  test.fixFirstPose(false);
  test.run();

  const auto& result = test.getResult();

  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose2(), result.at<gtsam::Pose2>(0)));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose2(gtsam::Rot2(0.1), gtsam::Point2(9.0, 0.09)),
      result.at<gtsam::Pose2>(9),
      1.0e-05));
}

/* ************************************************************************* */
TEST(RpgoGnc2D, LevenbergMarquardt) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::LEAST_SQUARES;
  auto lm_params = std::make_shared<LevenbergMarquardtParams>();
  config.solver_config.setLeastSquaresParams(lm_params);
  config.solver_config.setGncParamsDefault();

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_2d_robust.g2o";
  test.loadG2o(g2o_file, false);
  test.fixFirstPose(false);
  test.run();

  const auto& result = test.getResult();

  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose2(), result.at<gtsam::Pose2>(0)));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose2(gtsam::Rot2(0.1), gtsam::Point2(9.0, 0.09)),
      result.at<gtsam::Pose2>(9),
      1.0e-05));
}

/* ************************************************************************* */
TEST(RpgoGnc3D, GaussNewton) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::LEAST_SQUARES;
  auto gn_params = std::make_shared<GaussNewtonParams>();
  config.solver_config.setLeastSquaresParams(gn_params);
  config.solver_config.setGncParamsDefault();

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_3d_robust.g2o";
  test.loadG2o(g2o_file, true);
  test.fixFirstPose(true);
  test.run();

  const auto& result = test.getResult();

  gtsam::Key key0 = gtsam::Symbol('d', 0);
  gtsam::Key key9 = gtsam::Symbol('d', 9);
  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), result.at<gtsam::Pose3>(key0)));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(9, 0.09, -0.09)),
      result.at<gtsam::Pose3>(key9),
      1.0e-03));
}

/* ************************************************************************* */
TEST(RpgoGnc3D, LevenbergMarquardt) {
  RpgoConfig config;
  config.solver_type = RpgoConfig::SolverType::LEAST_SQUARES;
  auto lm_params = std::make_shared<LevenbergMarquardtParams>();
  config.solver_config.setLeastSquaresParams(lm_params);
  config.solver_config.setGncParamsDefault();

  Rpgo test(config);

  // load graph
  std::string g2o_file = std::string(DATASET_PATH) + "/test_3d_robust.g2o";
  test.loadG2o(g2o_file, true);
  test.fixFirstPose(true);
  test.run();

  const auto& result = test.getResult();

  gtsam::Key key0 = gtsam::Symbol('d', 0);
  gtsam::Key key9 = gtsam::Symbol('d', 9);
  EXPECT_EQ(result.size(), size_t(10));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), result.at<gtsam::Pose3>(key0)));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(9, 0.09, -0.09)),
      result.at<gtsam::Pose3>(key9),
      1.0e-03));
}

/* ************************************************************************* */
