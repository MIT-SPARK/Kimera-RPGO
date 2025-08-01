/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  pose_4dof.h
 * @brief 4DoF Pose
 * @author: Hyungtae Lim
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/dllexport.h>

#include <string>

namespace gtsam {

static const Eigen::MatrixBase<Eigen::Vector3d>::ConstantReturnType Z_1x3 =
    Eigen::Vector3d::Zero();

/**
 * A 3D Pose with fixed pitch and roll
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_UNSTABLE_EXPORT Pose4DoF : public LieGroup<Pose4DoF, 4> {
 public:
  static const size_t dimension = 4;

 protected:
  Pose2 T_;
  double z_;

  double pitch_;
  double roll_;

  mutable Point3 cached_translation_;  // Cached translation value
  mutable Rot3 cached_rotation_;       // Cached rotation value
  mutable Pose3 cached_pose_;          // Cached pose value
  // Flags indicating whether each cache is valid
  mutable bool cache_trans_valid_ = false;
  mutable bool cache_rot_valid_ = false;
  mutable bool cache_pose_valid_ = false;

  inline void updateTranslationCache() const {
    cached_translation_ = Point3(x(), y(), z());
    cache_trans_valid_ = true;
  }

  inline void updateRotationCache() const {
    cached_rotation_ = Rot3::Ypr(theta(), pitch_, roll_);
    cache_rot_valid_ = true;
  }

  inline void updatePoseCache() const {
    updateTranslationCache();
    updateRotationCache();
    cached_pose_ = Pose3(cached_rotation_, cached_translation_);
    cache_pose_valid_ = true;
  }

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor initializes at origin
  Pose4DoF() : T_(Pose2::Identity()), z_(0.0), pitch_(0.0), roll_(0.0) {}

  /// Copy constructor
  Pose4DoF(const Rot2& bearing, const Point3& t);
  Pose4DoF(const double x, const double y, const double z, const double theta);
  Pose4DoF(const double x,
           const double y,
           const double z,
           const double yaw,
           const double pitch,
           const double roll);
  Pose4DoF(const Pose2& pose, const double z);
  Pose4DoF(const Pose2& pose,
           const double z,
           const double pitch,
           const double roll);

  Pose4DoF(const Pose4DoF& pose);
  Pose4DoF(const Pose3& pose);

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const Pose4DoF& pose, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  inline double x() const { return T_.x(); }
  inline double y() const { return T_.y(); }
  inline double z() const { return z_; }
  inline double theta() const { return T_.theta(); }
  inline double yaw() const { return T_.theta(); }
  inline double pitch() const { return pitch_; }
  inline double roll() const { return roll_; }
  inline double c() const { return T_.r().c(); }
  inline double s() const { return T_.r().s(); }

  inline const Point2& translation2() const { return T_.t(); }
  inline const Rot2& rotation2() const { return T_.r(); }
  inline const Pose2& pose2() const { return T_; }

  inline const Point3& translation(OptionalJacobian<3, 4> Hself = {}) const {
    if (!cache_trans_valid_) updateTranslationCache();
    if (Hself) *Hself << c(), -s(), 0, 0, s(), c(), 0, 0, 0, 0, 1, 0;
    return cached_translation_;
  }

  inline const Rot3& rotation() const {
    if (!cache_rot_valid_) updateRotationCache();
    return cached_rotation_;
  }

  inline const Pose3& pose() const {
    if (!cache_pose_valid_) updatePoseCache();
    return cached_pose_;
  }

  /// @}
  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 4 DOF - used to autodetect sizes
  inline static size_t Dim() { return dimension; }

  /// Dimensionality of tangent space = 4 DOF
  inline size_t dim() const { return dimension; }

  /// Retraction from R^4 to Pose4DoF manifold neighborhood around current pose
  /// Tangent space parameterization is [x y z theta]
  // Pose4DoF retract(const Vector& v) const;

  /// Local 3D coordinates of Pose4DoF manifold neighborhood around current pose
  // Vector localCoordinates(const Pose4DoF& p2) const;

  // From ChatGPT
  Pose4DoF retract(const Vector& v,
                   OptionalJacobian<4, 4> Horigin,
                   OptionalJacobian<4, 4> Hv) const;

  Vector localCoordinates(const Pose4DoF& p2,
                          OptionalJacobian<4, 4> Horigin,
                          OptionalJacobian<4, 4> Hp2) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static Pose4DoF Identity() { return Pose4DoF(); }

  /// inverse transformation with derivatives
  Pose4DoF inverse(OptionalJacobian<4, 4> H1 = {}) const;

  /// compose this transformation onto another (first *this and then p2)
  Pose4DoF compose(const Pose4DoF& p2,
                   OptionalJacobian<4, 4> H1 = {},
                   OptionalJacobian<4, 4> H2 = {}) const;

  /// compose syntactic sugar
  inline Pose4DoF operator*(const Pose4DoF& T) const { return compose(T); }

  /**
   * Return relative pose between p1 and p2, in p1 coordinate frame
   * as well as optionally the derivatives
   */
  Pose4DoF between(const Pose4DoF& p2,
                   OptionalJacobian<4, 4> H1 = {},
                   OptionalJacobian<4, 4> H2 = {}) const;

  /// @}
  /// @name Group Action on Point2
  /// @{

  /** Return point coordinates in pose coordinate frame */
  Point3 transformTo(const Point3& point,
                     OptionalJacobian<3, 4> Hpose = {},
                     OptionalJacobian<3, 3> Hpoint = {}) const;

  /// Exponential map at identity - create a rotation from canonical coordinates
  static Pose4DoF Expmap(const Vector& xi);

  /// Log map at identity - return the canonical coordinates of this rotation
  static Vector Logmap(const Pose4DoF& p);

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  //
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(T_);
    ar& BOOST_SERIALIZATION_NVP(z_);
  }
#endif
};  // \class Pose4DoF

template <>
struct traits<Pose4DoF> : public internal::LieGroup<Pose4DoF> {
  using ChartJacobian = OptionalJacobian<4, 4>;

  static Pose4DoF Identity() { return Pose4DoF(); }

  // NOTE(hlim) In PGO, this function is not used
  // static Pose4DoF Compose(const Pose4DoF& g, const Pose4DoF& h,
  //                             OptionalJacobian<4, 4> H1 = {},
  //                             OptionalJacobian<4, 4> H2 = {}) {
  //   return g.compose(h, H1, H2);
  // }

  static Pose4DoF Between(const Pose4DoF& g,
                          const Pose4DoF& h,
                          OptionalJacobian<4, 4> H1 = {},
                          OptionalJacobian<4, 4> H2 = {}) {
    return g.between(h, H1, H2);
  }

  // NOTE(hlim) In PGO, this function is not used
  // static Pose4DoF Inverse(const Pose4DoF& g,
  //                             OptionalJacobian<4, 4> H = {}) {
  //   return g.inverse(H);
  // }

  static Pose4DoF Expmap(const Vector& xi, OptionalJacobian<4, 4> /* H */ = {}) {
    return Pose4DoF::Expmap(xi);
  }

  static Vector Logmap(const Pose4DoF& g, OptionalJacobian<4, 4> /* H */ = {}) {
    return Pose4DoF::Logmap(g);
  }
};

template <>
struct traits<const Pose4DoF> : public traits<Pose4DoF> {};

}  // namespace gtsam
