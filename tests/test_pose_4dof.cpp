#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>

#include "kimera_rpgo/utils/pose_4dof.h"

namespace gtsam {
namespace {

TEST(Pose4DoF, FixedTiltRetractionAndRelativePoseDerivatives) {
  const Pose4DoF first(1.0, 2.0, 3.0, 0.7, 0.2, -0.3);
  const Pose4DoF second(-0.5, 0.6, 2.1, -0.4, -0.1, 0.5);
  const Vector4 delta = (Vector4() << 0.2, -0.1, 0.3, 0.15).finished();
  Matrix44 H1, H2;
  const auto moved = first.retract(delta, H1, H2);
  EXPECT_DOUBLE_EQ(moved.pitch(), first.pitch());
  EXPECT_DOUBLE_EQ(moved.roll(), first.roll());
  EXPECT_TRUE(traits<Pose4DoF>::Local(first, moved).isApprox(delta));
  const std::function<Pose4DoF(const Pose4DoF&, const Vector4&)> retract =
      [](const auto& pose, const auto& increment) {
        return traits<Pose4DoF>::Retract(pose, increment);
      };
  EXPECT_TRUE(H1.isApprox((numericalDerivative21<Pose4DoF, Pose4DoF, Vector4>(
                              retract, first, delta)),
                          1e-6));
  EXPECT_TRUE(H2.isApprox((numericalDerivative22<Pose4DoF, Pose4DoF, Vector4>(
                              retract, first, delta)),
                          1e-6));

  first.between(second, H1, H2);
  const std::function<Pose4DoF(const Pose4DoF&, const Pose4DoF&)> between =
      [](const auto& a, const auto& b) { return a.between(b); };
  EXPECT_TRUE(H1.isApprox((numericalDerivative21<Pose4DoF, Pose4DoF, Pose4DoF>(
                              between, first, second)),
                          1e-6));
  EXPECT_TRUE(H2.isApprox((numericalDerivative22<Pose4DoF, Pose4DoF, Pose4DoF>(
                              between, first, second)),
                          1e-6));

  first.localCoordinates(second, H1, H2);
  const std::function<Vector4(const Pose4DoF&, const Pose4DoF&)> local =
      [](const auto& a, const auto& b) {
        return traits<Pose4DoF>::Local(a, b);
      };
  EXPECT_TRUE(H1.isApprox((numericalDerivative21<Vector4, Pose4DoF, Pose4DoF>(
                              local, first, second)),
                          1e-6));
  EXPECT_TRUE(H2.isApprox((numericalDerivative22<Vector4, Pose4DoF, Pose4DoF>(
                              local, first, second)),
                          1e-6));
}

TEST(Pose4DoF, TiltedPointTransformDerivative) {
  const Pose4DoF pose(1.0, 2.0, 3.0, 0.7, 0.2, -0.3);
  const Point3 point(0.4, -0.2, 1.3);
  Matrix34 H;
  pose.transformTo(point, H);
  const std::function<Point3(const Pose4DoF&)> transform =
      [&](const auto& value) { return value.transformTo(point); };
  EXPECT_TRUE(H.isApprox(
      (numericalDerivative11<Point3, Pose4DoF>(transform, pose)), 1e-6));
}

TEST(Pose4DoF, EqualityIncludesFixedTilt) {
  const Pose4DoF pose(1.0, 2.0, 3.0, 0.7, 0.2, -0.3);
  EXPECT_TRUE(pose.equals(Pose4DoF(1.0, 2.0, 3.0, 0.7, 0.2, -0.3)));
  EXPECT_FALSE(pose.equals(Pose4DoF(1.0, 2.0, 3.0, 0.7, 0.0, -0.3)));
  EXPECT_FALSE(pose.equals(Pose4DoF(1.0, 2.0, 3.0, 0.7, 0.2, 0.0)));
}

}  // namespace
}  // namespace gtsam
