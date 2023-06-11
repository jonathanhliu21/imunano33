/**
 * @file
 *
 * @brief File containing Quaternion class
 */

#ifndef INCLUDE_IMUNANO33_QUATERNION_HPP_
#define INCLUDE_IMUNANO33_QUATERNION_HPP_

#include <cmath>

#include "imunano33/simplevectors.hpp"

namespace imunano33 {
using svector::Vector3D;

/**
 * @brief A simple quaternion class for rotations
 */
class Quaternion {
public:
  /**
   * @brief Default constructor
   *
   * Initializes quaternion to [1, 0, 0, 0]
   */
  Quaternion() : m_w{1} {}

  /**
   * @brief Constructor for a basic quaternion
   *
   * @param w The scalar component
   * @param vec The vector component
   */
  Quaternion(const double w, Vector3D vec) : m_vec{vec} {
    // makes sure that quaternion magnitude is nonzero
    // this is important for rotations
    if (w == 0 && isZero(vec)) {
      m_w = 1;
    } else {
      m_w = w;
    }
  }

  /**
   * @brief Constructor for a rotation quaternion
   *
   * @param vec The vector to rotate around
   * @param ang The angle to rotate around by
   *
   * @note A zero vector passed into vec will result in undefined behavior
   */
  Quaternion(Vector3D vec, const double ang) {
    Vector3D norm = vec.normalize();
    m_w = std::cos(ang / 2);

    m_vec.x(norm.x() * std::sin(ang / 2));
    m_vec.y(norm.y() * std::sin(ang / 2));
    m_vec.z(norm.z() * std::sin(ang / 2));
  }

  /**
   * @brief Copy constructor
   *
   * @param other Other quaternion
   */
  Quaternion(const Quaternion &other) : m_w{other.m_w}, m_vec{other.m_vec} {}

  /**
   * @brief Assignment operator
   */
  Quaternion &operator=(const Quaternion &other) {
    if (this == &other) {
      return *this;
    }

    m_w = other.m_w;
    m_vec = other.m_vec;

    return *this;
  }

  /**
   * @brief Gets the scalar component of the quaternion
   *
   * @returns The scalar component
   */
  double GetW() const { return m_w; }

  /**
   * @brief Gets the vector component of the quaternion
   *
   * @returns The vector component
   */
  Vector3D GetVec() const { return m_vec; }

  /**
   * @brief Gets the quaternion conjugate
   *
   * @returns The quaternion conjugate
   */
  Quaternion Conjugate() const { return Quaternion{m_w, -m_vec}; }

  /**
   * @brief Gets quaternion inverse
   *
   * @returns Quaternion inverse
   */
  Quaternion Inverse() const {
    Quaternion conj = Conjugate();
    double magn = Magn();
    double newW = conj.GetW() / (magn * magn);
    Vector3D newVec = conj.GetVec() / (magn * magn);

    return Quaternion{newW, newVec};
  }

  /**
   * @brief Gets quaternion magnitude
   *
   * @returns Quaternion magnitude
   */
  double Magn() const {
    return std::sqrt(m_w * m_w + m_vec.x() * m_vec.x() + m_vec.y() * m_vec.y() +
                     m_vec.z() * m_vec.z());
  }

  /**
   * @brief Gets normalized quaternion
   *
   * @note If the quaternion is zero, then results in undefined behavior.
   *
   * @returns Normalized quaternion
   */
  Quaternion Normalize() const {
    double magn = Magn();
    double newW = m_w / magn;
    Vector3D newVec = m_vec / magn;

    return Quaternion{newW, newVec};
  }

  static Vector3D
  Rotate(const Vector3D &vec, const Vector3D &axis,
         const double ang); // defined later, where operators are defined

private:
  double m_w;
  Vector3D m_vec;
};

/**
 * @brief Product of two quaternions
 *
 * @param lhs Left hand argument
 * @param rhs Right hand argument
 * '
 * @returns Product
 */
inline Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs) {
  using svector::Vector3D;

  double wl = lhs.GetW();
  double wr = rhs.GetW();

  Vector3D vl = lhs.GetVec();
  Vector3D vr = rhs.GetVec();

  return Quaternion(wl * wr - vl.dot(vr), vr * wl + vl * wr + vl.cross(vr));
}

/**
 * @brief Equality of two quaternions
 *
 * @param lhs Left hand argument
 * @param rhs Right hand argument
 *
 * @returns Whether the quaternions are equal
 */
inline bool operator==(const Quaternion &lhs, const Quaternion &rhs) {
  return lhs.GetW() == rhs.GetW() && lhs.GetVec() == rhs.GetVec();
}

/**
 * @brief Inequality of two quaternions
 *
 * @param lhs Left hand argument
 * @param rhs Right hand argument
 *
 * @returns Whether the two quaternions are not equal
 */
inline bool operator!=(const Quaternion &lhs, const Quaternion &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Rotates a vector
 *
 * @param vec The vector to rotate
 * @param axis The axis of rotation
 * @param ang Angle of rotation
 *
 * @returns The rotated vector.
 */
inline Vector3D Quaternion::Rotate(const Vector3D &vec, const Vector3D &axis,
                                   const double ang) {
  Quaternion rotationQuat{axis.normalize(), ang};
  Quaternion vecQuat{0, vec};
  Quaternion res = rotationQuat * vecQuat * rotationQuat.Conjugate();

  return res.GetVec();
}
} // namespace imunano33

#endif
