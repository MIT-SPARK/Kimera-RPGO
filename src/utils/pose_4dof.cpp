/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  pose_4dof.cpp
 * @brief 4DoF Pose
 * @author: Hyungtae Lim
 */

#include "kimera_rpgo/utils/pose_4dof.h"

#include <iostream>

#include "gtsam/base/OptionalJacobian.h"
#include "gtsam/base/Vector.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const Rot2& bearing, const Point3& t)
    : T_(bearing, Point2(t.x(), t.y())), z_(t.z()), pitch_(0.0), roll_(0.0) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const double x,
                   const double y,
                   const double z,
                   const double theta)
    : T_(x, y, theta), z_(z), pitch_(0.0), roll_(0.0) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const double x,
                   const double y,
                   const double z,
                   const double yaw,
                   const double pitch,
                   const double roll)
    : T_(x, y, yaw), z_(z), pitch_(pitch), roll_(roll) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const Pose2& pose, const double z)
    : T_(pose), z_(z), pitch_(0.0), roll_(0.0) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const Pose2& pose,
                   const double z,
                   const double pitch,
                   const double roll)
    : T_(pose), z_(z), pitch_(pitch), roll_(roll) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const Pose4DoF& pose)
    : T_(pose.T_), z_(pose.z_), pitch_(pose.pitch_), roll_(pose.roll_) {}

/* ************************************************************************* */
Pose4DoF::Pose4DoF(const Pose3& pose) {
  const auto ypr = pose.rotation().ypr();
  T_ = Pose2(pose.x(), pose.y(), ypr(0));
  z_ = pose.z();
  pitch_ = ypr(1);
  roll_ = ypr(2);
}

/* ************************************************************************* */
void Pose4DoF::print(const std::string& s) const {
  cout << s << "(" << T_.x() << ", " << T_.y() << ", " << z_ << ", ";
  cout << T_.theta() << ", " << pitch_ << ", " << roll_ << ")\n";
  cout << "R: " << rotation() << endl;
}

/* ************************************************************************* */
bool Pose4DoF::equals(const Pose4DoF& x, double tol) const {
  return T_.equals(x.T_, tol) && std::abs(z_ - x.z_) < tol;
}

/* ************************************************************************* */
// NOTE(hlim) In PGO, this function is not used
Pose4DoF Pose4DoF::inverse(OptionalJacobian<4, 4> H1) const {
  if (!H1) {
    return Pose4DoF(T_.inverse(), -z_);
  }
  OptionalJacobian<3, 3>::Jacobian H3x3;
  // TODO(kartikarcot): Could not use reference to a view into H1 and reuse
  // memory Eigen::Ref<Eigen::Matrix<double, 3, 3>> H3x3 =
  // H1->topLeftCorner(3,3);
  Pose4DoF result(T_.inverse(H3x3), -z_);
  Matrix H1_ = -I_4x4;
  H1_.topLeftCorner(2, 2) = H3x3.topLeftCorner(2, 2);
  H1_.topRightCorner(2, 1) = H3x3.topRightCorner(2, 1);
  *H1 = H1_;
  return result;
}

/* ************************************************************************* */
// NOTE(hlim) In PGO, this function is not used
Pose4DoF Pose4DoF::compose(const Pose4DoF& p2,
                           OptionalJacobian<4, 4> H1,
                           OptionalJacobian<4, 4> H2) const {
  if (!H1 && !H2) return Pose4DoF(pose().compose(p2.pose()));

  // TODO(kartikarcot): Could not use reference to a view into H1 and reuse
  // memory
  OptionalJacobian<3, 3>::Jacobian H3x3;
  // Pose4DoF(pose().compose(p2.pose()));
  Pose4DoF result(T_.compose(p2.T_, H3x3), z_ + p2.z_);
  if (H1) {
    Matrix H1_ = I_4x4;
    H1_.topLeftCorner(2, 2) = H3x3.topLeftCorner(2, 2);
    H1_.topRightCorner(2, 1) = H3x3.topRightCorner(2, 1);
    *H1 = H1_;
  }
  if (H2) *H2 = I_4x4;
  return result;
}

/* ************************************************************************* */
Pose4DoF Pose4DoF::between(const Pose4DoF& p2,
                           OptionalJacobian<4, 4> H1,
                           OptionalJacobian<4, 4> H2) const {
  if (!H1 && !H2) return Pose4DoF(T_.between(p2.T_), p2.z_ - z_);

  OptionalJacobian<3, 3>::Jacobian H3x3_1, H3x3_2;
  Pose4DoF result(T_.between(p2.T_, H3x3_1, H3x3_2), p2.z_ - z_);
  if (H1) {
    H1->topLeftCorner(2, 2) = H3x3_1.topLeftCorner(2, 2);
    H1->topRightCorner(2, 1) = H3x3_1.topRightCorner(2, 1);
    (*H1)(0, 2) = 0.0;
    (*H1)(1, 2) = 0.0;
    H1->row(2) << 0.0, 0.0, -1.0, 0.0;
    H1->row(3) << 0.0, 0.0, 0.0, -1.0;
  }
  if (H2) *H2 = I_4x4;
  return result;
}

Point3 Pose4DoF::transformTo(const Point3& point,
                             OptionalJacobian<3, 4> Hpose,
                             OptionalJacobian<3, 3> Hpoint) const {
  // Decompose the pose
  const gtsam::Point2& t = T_.translation();
  const double z = z_;

  // Compute the relative point in local coordinates
  gtsam::Point3 relative_point(
      point.x() - t.x(), point.y() - t.y(), point.z() - z);

  const gtsam::Rot3& R = rotation();
  gtsam::Point3 q = R.unrotate(relative_point);

  // NOTE(hlim) The Jacobian of the 4DoF factor is computed as:
  // H matrix = -t + [R^T (p - t)] × (R_p R_r)^T * \theta
  double cp = cos(pitch());
  double sp = sin(pitch());
  double cr = cos(roll());
  double sr = sin(roll());

  double RrRpT13 = -sp;
  double RrRpT23 = cp * sr;
  double RrRpT33 = cp * cr;

  double a13 = -q.z() * RrRpT23 + q.y() * RrRpT33;
  double a23 = q.z() * RrRpT13 - q.x() * RrRpT33;
  double a33 = -q.y() * RrRpT13 + q.x() * RrRpT23;

  if (Hpose) {
    *Hpose << -1.0, 0.0, 0.0, a13, 0.0, -1.0, 0.0, a23, 0.0, 0.0, -1.0, a33;
  }

  if (Hpoint) {
    *Hpoint << R.transpose().matrix();
  }

  return q;
}

Pose4DoF Pose4DoF::retract(const Vector& v,
                           OptionalJacobian<4, 4> Horigin,
                           OptionalJacobian<4, 4> Hv) const {
  assert(v.size() == 4);
  Vector v2(3);
  v2 << v(0), v(1), v(3);

  OptionalJacobian<3, 3>::Jacobian H3x3_1, H3x3_2;
  Pose2 retractedPose2 = T_.retract(v2, H3x3_1, H3x3_2);

  double z = z_ + v(2);

  if (Horigin) {
    // TODO(hlim) This matrix should be double checked
    *Horigin = I_4x4;
    Horigin->topLeftCorner<2, 2>() = H3x3_1.topLeftCorner<2, 2>();
    Horigin->topRightCorner<2, 1>() = H3x3_1.topRightCorner<2, 1>();
    Horigin->col(2) << 0, 0, 1, 0;
  }

  if (Hv) {
    // TODO(hlim) This matrix should be double checked
    *Hv = I_4x4;
    Hv->topLeftCorner<2, 2>() = H3x3_2.topLeftCorner<2, 2>();
    Hv->topRightCorner<2, 1>() = H3x3_2.topRightCorner<2, 1>();
    Hv->col(2) << 0, 0, 1, 0;
  }

  return Pose4DoF(retractedPose2, z, pitch_, roll_);
}

Vector Pose4DoF::localCoordinates(const Pose4DoF& p2,
                                  OptionalJacobian<4, 4> Horigin,
                                  OptionalJacobian<4, 4> Hp2) const {
  OptionalJacobian<3, 3>::Jacobian H3x3_1, H3x3_2;
  Vector pose2 = T_.localCoordinates(p2.pose2(), H3x3_1, H3x3_2);

  Vector result(4);
  result << pose2(0), pose2(1), p2.z() - z_, pose2(2);

  if (Horigin) {
    // TODO(hlim) This matrix should be double checked
    *Horigin = -I_4x4;
    Horigin->topLeftCorner<2, 2>() = H3x3_1.topLeftCorner<2, 2>();
    Horigin->topRightCorner<2, 1>() = H3x3_1.topRightCorner<2, 1>();
    Horigin->col(2) << 0, 0, -1, 0;
  }

  if (Hp2) {
    // TODO(hlim) This matrix should be double checked
    *Hp2 = I_4x4;
    Hp2->topLeftCorner<2, 2>() = H3x3_2.topLeftCorner<2, 2>();
    Hp2->topRightCorner<2, 1>() = H3x3_2.topRightCorner<2, 1>();
    Hp2->col(2) << 0, 0, 1, 0;
  }

  return result;
}

/* ************************************************************************* */
Pose4DoF Pose4DoF::Expmap(const Vector& xi) {
  assert(xi.size() == 4);
  Vector v1(3);
  v1 << xi(0), xi(1), xi(3);
  return Pose4DoF(Pose2::Expmap(v1), xi(2));
}

/* ************************************************************************* */
Vector Pose4DoF::Logmap(const Pose4DoF& p) {
  Vector pose2 = Pose2::Logmap(p.pose2());
  Vector result(4);
  result << pose2(0), pose2(1), p.z(), pose2(2);
  return result;
}
/* ************************************************************************* */

}  // namespace gtsam
