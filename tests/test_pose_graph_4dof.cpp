#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/PriorFactor.h>

#include "kimera_rpgo/pose_graph_4dof.h"
#include "kimera_rpgo/rpgo.h"

namespace kimera_rpgo {
namespace {

using gtsam::Pose3;
using gtsam::Pose4DoF;

TEST(PoseGraph4DoF, NoiseProjectionKeepsYawVarianceAndCorrelations) {
  gtsam::Matrix6 covariance = gtsam::Matrix6::Identity();
  covariance.diagonal() << 0.02, 0.03, 0.07, 0.1, 0.2, 0.3;
  covariance(2, 3) = covariance(3, 2) = 0.01;
  const auto noise = projectBetweenNoise(
      Pose3(), Pose3(), gtsam::noiseModel::Gaussian::Covariance(covariance));
  const auto projected =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(noise.get())
          ->covariance();
  gtsam::Matrix4 expected = gtsam::Matrix4::Zero();
  expected.diagonal() << 0.1, 0.2, 0.3, 0.07;
  expected(0, 3) = expected(3, 0) = 0.01;
  EXPECT_TRUE(projected.isApprox(expected, 1e-6));
}

TEST(PoseGraph4DoF, BetweenDerivativeAtNonzeroResidual) {
  const Pose4DoF from(1, 2, 3, 0.7, 0.2, -0.3);
  const Pose4DoF to(2, -1, 0.5, -0.4, -0.1, 0.4);
  const Pose4BetweenFactor factor(
      0,
      1,
      Pose4DoF(0.2, 0.3, 0.1, 0.5),
      gtsam::noiseModel::Isotropic::Variance(4, 0.1));
  gtsam::Matrix H1, H2;
  factor.evaluateError(from, to, H1, H2);
  const std::function<gtsam::Vector4(const Pose4DoF&, const Pose4DoF&)> error =
      [&](const auto& a, const auto& b) { return factor.evaluateError(a, b); };
  EXPECT_TRUE(H1.isApprox(
      (gtsam::numericalDerivative21<gtsam::Vector4, Pose4DoF, Pose4DoF>(
          error, from, to)),
      1e-6));
  EXPECT_TRUE(H2.isApprox(
      (gtsam::numericalDerivative22<gtsam::Vector4, Pose4DoF, Pose4DoF>(
          error, from, to)),
      1e-6));
}

TEST(PoseGraph4DoF, ProjectionUsesGravityAlignedSourceFrame) {
  const Pose3 source(gtsam::Rot3::Ypr(0.7, 0.2, -0.3), {1.0, 2.0, 3.0});
  const Pose3 measurement(gtsam::Rot3::Ypr(-0.2, 0.1, 0.4), {0.5, -0.2, 0.7});
  const auto target = source.compose(measurement);
  const auto projected = projectBetween(source, measurement);
  const auto expected_translation =
      gtsam::Rot3::Yaw(source.rotation().yaw())
          .unrotate(target.translation() - source.translation());
  EXPECT_TRUE(projected.translation().isApprox(expected_translation, 1e-9));
  EXPECT_NEAR(
      projected.yaw(), target.rotation().yaw() - source.rotation().yaw(), 1e-9);
}

TEST(PoseGraph4DoF, Pose3EstimatesPreserveMixedValuesAndTilt) {
  const Pose3 full(gtsam::Rot3::Ypr(0.2, -0.4, 0.3), {1.0, 2.0, 3.0});
  const Pose4DoF reduced(2.0, -1.0, 0.5, 0.7, 0.2, -0.3);
  gtsam::Values values;
  values.insert(7, full);
  values.insert(42, reduced);
  const auto converted = pose3Estimates(values);
  ASSERT_EQ(converted.size(), 2u);
  EXPECT_TRUE(converted.at<Pose3>(7).equals(full));
  EXPECT_TRUE(converted.at<Pose3>(42).equals(reduced.pose()));
}

TEST(PoseGraph4DoF, RpgoOptimizesNativeGraphAndRejectsPcm) {
  const Pose4DoF first(1.0, 2.0, 3.0, 0.7, 0.2, -0.3);
  const Pose4DoF second(2.0, -1.0, 0.5, -0.4, -0.1, 0.4);
  const auto noise = gtsam::noiseModel::Isotropic::Variance(4, 0.01);
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::PriorFactor<Pose4DoF>(0, first, noise));
  factors.add(Pose4BetweenFactor(0, 1, first.between(second), noise));
  gtsam::Values initial;
  initial.insert(0, first);
  const auto delta = (gtsam::Vector4() << 0.4, -0.2, 0.3, 0.2).finished();
  initial.insert(1, second.retract(delta, {}, {}));
  RpgoConfig config;
  config.print_summary = false;
  config.solver_config.setLeastSquaresParamsDefault();
  Rpgo optimizer(config, false);
  optimizer.addFactors(factors);
  optimizer.addValues(initial);
  optimizer.run();
  ASSERT_EQ(optimizer.getResult().size(), 2u);
  EXPECT_TRUE(optimizer.getResult().at<Pose4DoF>(0).equals(first, 1e-6));
  EXPECT_TRUE(optimizer.getResult().at<Pose4DoF>(1).equals(second, 1e-6));

  config.use_pcm = true;
  Rpgo pcm(config, false);
  pcm.addFactors(factors);
  pcm.addValues(initial);
  EXPECT_THROW(pcm.run(), std::invalid_argument);
}

}  // namespace
}  // namespace kimera_rpgo
