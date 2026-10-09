#include "kimera_rpgo/pose_graph_4dof.h"

#include <gtsam/base/numericalDerivative.h>

#include <stdexcept>

namespace kimera_rpgo {

gtsam::Vector Pose4BetweenFactor::evaluateError(const gtsam::Pose4DoF& from,
                                                const gtsam::Pose4DoF& to,
                                                GtsamJacobianType H1,
                                                GtsamJacobianType H2) const {
  gtsam::Matrix44 between_from, between_to, local;
  const auto relative = from.between(to, between_from, between_to);
  const auto error = measured().localCoordinates(relative, {}, local);
  if (H1) {
    *H1 = local * between_from;
  }

  if (H2) {
    *H2 = local * between_to;
  }

  return error;
}

gtsam::NonlinearFactor::shared_ptr Pose4BetweenFactor::clone() const {
  return boost::make_shared<Pose4BetweenFactor>(*this);
}

gtsam::Values pose3Estimates(const gtsam::Values& values) {
  gtsam::Values result;
  for (const auto& entry : values) {
    if (const auto value =
            dynamic_cast<const gtsam::GenericValue<gtsam::Pose4DoF>*>(
                &entry.value)) {
      result.insert(entry.key, value->value().pose());
    } else {
      result.insert(entry.key, entry.value.cast<gtsam::Pose3>());
    }
  }

  return result;
}

gtsam::Pose4DoF projectBetween(const gtsam::Pose3& source,
                               const gtsam::Pose3& measurement) {
  return gtsam::Pose4DoF(source).between(
      gtsam::Pose4DoF(source.compose(measurement)));
}

gtsam::SharedNoiseModel projectBetweenNoise(
    const gtsam::Pose3& source,
    const gtsam::Pose3& measurement,
    const gtsam::SharedNoiseModel& noise) {
  const auto gaussian =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(noise.get());
  if (!gaussian || gaussian->dim() != 6) {
    throw std::invalid_argument(
        "Pose projection requires six-dimensional Gaussian noise");
  }

  const std::function<gtsam::Pose4DoF(const gtsam::Pose3&)> projection =
      [&](const gtsam::Pose3& pose) { return projectBetween(source, pose); };
  const auto jacobian =
      gtsam::numericalDerivative11<gtsam::Pose4DoF, gtsam::Pose3>(projection,
                                                                  measurement);
  return gtsam::noiseModel::Gaussian::Covariance(
      jacobian * gaussian->covariance() * jacobian.transpose());
}

}  // namespace kimera_rpgo
