#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "kimera_rpgo/utils/pose_4dof.h"

namespace kimera_rpgo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#endif

//! Between factor with complete local-coordinate Jacobians.
class Pose4BetweenFactor : public gtsam::BetweenFactor<gtsam::Pose4DoF> {
 public:
  using gtsam::BetweenFactor<gtsam::Pose4DoF>::BetweenFactor;

  gtsam::Vector evaluateError(const gtsam::Pose4DoF& from,
                              const gtsam::Pose4DoF& to,
                              GtsamJacobianType H1 = {},
                              GtsamJacobianType H2 = {}) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

//! Convert mixed pose values to Pose3, preserving keys and fixed tilt.
gtsam::Values pose3Estimates(const gtsam::Values& values);

//! Project a full relative pose into the gravity-aligned source frame.
gtsam::Pose4DoF projectBetween(const gtsam::Pose3& source,
                               const gtsam::Pose3& measurement);

//! Project Pose3 covariance into the four-dimensional tangent space.
gtsam::SharedNoiseModel projectBetweenNoise(
    const gtsam::Pose3& source,
    const gtsam::Pose3& measurement,
    const gtsam::SharedNoiseModel& noise);

}  // namespace kimera_rpgo
