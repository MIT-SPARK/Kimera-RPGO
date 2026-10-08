#include "kimera_rpgo/utils/utils.h"

#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values-inl.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

#include <fstream>

namespace kimera_rpgo {
using gtsam::BetweenFactor;
using gtsam::GenericValue;
using gtsam::Matrix2;
using gtsam::Matrix3;
using gtsam::Matrix6;
using gtsam::NonlinearFactorGraph;
using gtsam::Point2;
using gtsam::Point3;
using gtsam::Pose2;
using gtsam::Pose3;
using gtsam::PoseToPointFactor;
using gtsam::SharedNoiseModel;
using gtsam::Values;

using std::cout;
using std::endl;
using std::fstream;
using std::invalid_argument;

void saveG2o(const NonlinearFactorGraph& graph,
             const Values& estimate,
             const std::string& filename) {
  fstream stream(filename.c_str(), fstream::out);

  // save 2D poses
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Pose2>*>(&key_value.value);
    if (!p) continue;
    const Pose2& pose = p->value();
    stream << "VERTEX_SE2 " << key_value.key << " " << pose.x() << " "
           << pose.y() << " " << pose.theta() << endl;
  }

  // save 3D poses
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
    if (!p) continue;
    const Pose3& pose = p->value();
    const Point3 t = pose.translation();
    const auto q = pose.rotation().toQuaternion();
    stream << "VERTEX_SE3:QUAT " << key_value.key << " " << t.x() << " "
           << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
           << q.z() << " " << q.w() << endl;
  }

  // save 2D landmarks
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Point2>*>(&key_value.value);
    if (!p) continue;
    const Point2& point = p->value();
    stream << "VERTEX_XY " << key_value.key << " " << point.x() << " "
           << point.y() << endl;
  }

  // save 3D landmarks
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Point3>*>(&key_value.value);
    if (!p) continue;
    const Point3& point = p->value();
    stream << "VERTEX_TRACKXYZ " << key_value.key << " " << point.x() << " "
           << point.y() << " " << point.z() << endl;
  }

  // save edges (2D or 3D)
  for (const auto& factor_ : graph) {
    auto between2D = factor_pointer_cast<BetweenFactor<Pose2>>(factor_);
    if (between2D) {
      SharedNoiseModel model = between2D->noiseModel();
      auto gaussianModel =
          factor_pointer_cast<gtsam::noiseModel::Gaussian>(model);
      auto robustModel = factor_pointer_cast<gtsam::noiseModel::Robust>(model);
      if (robustModel) {
        gaussianModel = factor_pointer_cast<gtsam::noiseModel::Gaussian>(
            robustModel->noise());
      }

      if (!gaussianModel) {
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }

      Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
      Pose2 pose = between2D->measured();  //.inverse();
      stream << "EDGE_SE2 " << between2D->key1() << " " << between2D->key2()
             << " " << pose.x() << " " << pose.y() << " " << pose.theta();
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = i; j < 3; j++) {
          stream << " " << Info(i, j);
        }
      }

      stream << endl;
    }

    auto between3D = factor_pointer_cast<BetweenFactor<Pose3>>(factor_);
    if (between3D) {
      SharedNoiseModel model = between3D->noiseModel();
      auto gaussianModel =
          factor_pointer_cast<gtsam::noiseModel::Gaussian>(model);
      auto robustModel = factor_pointer_cast<gtsam::noiseModel::Robust>(model);
      if (robustModel) {
        gaussianModel = factor_pointer_cast<gtsam::noiseModel::Gaussian>(
            robustModel->noise());
      }

      if (!gaussianModel) {
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }

      Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
      const Pose3 pose3D = between3D->measured();
      const Point3 p = pose3D.translation();
      const auto q = pose3D.rotation().toQuaternion();
      stream << "EDGE_SE3:QUAT " << between3D->key1() << " "
             << between3D->key2() << " " << p.x() << " " << p.y() << " "
             << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
             << q.w();

      Matrix6 InfoG2o = Eigen::MatrixXd::Identity(6, 6);
      InfoG2o.block<3, 3>(0, 0) = Info.block<3, 3>(3, 3);  // cov translation
      InfoG2o.block<3, 3>(3, 3) = Info.block<3, 3>(0, 0);  // cov rotation
      InfoG2o.block<3, 3>(0, 3) = Info.block<3, 3>(0, 3);  // off diagonal
      InfoG2o.block<3, 3>(3, 0) = Info.block<3, 3>(3, 0);  // off diagonal

      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          stream << " " << InfoG2o(i, j);
        }
      }

      stream << endl;
    }

    auto landmark2D =
        factor_pointer_cast<PoseToPointFactor<Pose2, Point2>>(factor_);
    if (landmark2D) {
      SharedNoiseModel model = landmark2D->noiseModel();
      auto gaussianModel =
          factor_pointer_cast<gtsam::noiseModel::Gaussian>(model);
      auto robustModel = factor_pointer_cast<gtsam::noiseModel::Robust>(model);
      if (robustModel) {
        gaussianModel = factor_pointer_cast<gtsam::noiseModel::Gaussian>(
            robustModel->noise());
      }

      if (!gaussianModel) {
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }

      Matrix2 Info = gaussianModel->R().transpose() * gaussianModel->R();
      Point2 pos = landmark2D->measured();  //.inverse();
      stream << "EDGE_SE2_XY " << landmark2D->key1() << " "
             << landmark2D->key2() << " " << pos.x() << " " << pos.y();
      for (size_t i = 0; i < 2; i++) {
        for (size_t j = i; j < 2; j++) {
          stream << " " << Info(i, j);
        }
      }

      stream << endl;
    }

    // TODO(Yun) add landmark 3D and others
  }

  stream.close();
}

}  // namespace kimera_rpgo
