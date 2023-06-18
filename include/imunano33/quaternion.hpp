/**
 * @file
 *
 * @brief File containing the imunano33::Quaternion class
 */

#ifndef INCLUDE_IMUNANO33_QUATERNION_HPP_
#define INCLUDE_IMUNANO33_QUATERNION_HPP_

#include <cmath>

#include "imunano33/simplevectors.hpp"

namespace imunano33 {
using svector::Vector3D;

/**
 * @brief A simple quaternion class for rotations
 *
 * The quaternion operations and math are mainly based on this paper:
 * https://jerabaul29.github.io/assets/quaternions/quaternions.pdf
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
    Vector3D norm = normalize(vec);
    m_w = std::cos(ang / 2);

    x(m_vec, x(norm) * std::sin(ang / 2));
    y(m_vec, y(norm) * std::sin(ang / 2));
    z(m_vec, z(norm) * std::sin(ang / 2));
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
  double w() const { return m_w; }

  /**
   * @brief Gets the vector component of the quaternion
   *
   * @returns The vector component
   */
  Vector3D vec() const { return m_vec; }

  /**
   * @brief Gets the quaternion conjugate
   *
   * @returns The quaternion conjugate
   */
  Quaternion conj() const { return Quaternion{m_w, -m_vec}; }

  /**
   * @brief Gets quaternion inverse
   *
   * @returns Quaternion inverse
   */
  Quaternion inv() const {
    Quaternion conju = conj();
    double mag = norm();
    double newW = conju.w() / (mag * mag);
    Vector3D newVec = conju.vec() / (mag * mag);

    return Quaternion{newW, newVec};
  }

  /**
   * @brief Gets quaternion norm
   *
   * This behaves the same as a "magnitude" in 4-dimensional vector terms.
   *
   * @returns Quaternion norm
   */
  double norm() const {
    return std::sqrt(m_w * m_w + x(m_vec) * x(m_vec) + y(m_vec) * y(m_vec) +
                     z(m_vec) * z(m_vec));
  }

  /**
   * @brief Gets equivalent unit quaternion
   *
   * This behaves the same as a "normalized" 4-dimensional vector.
   *
   * @note If the quaternion is zero, then results in undefined behavior.
   *
   * @returns Equivalent unit quaternion
   */
  Quaternion unit() const {
    double mag = norm();
    double newW = m_w / mag;
    Vector3D newVec = m_vec / mag;

    return Quaternion{newW, newVec};
  }

  // defined later, where operators are defined
  Quaternion &operator*=(const Quaternion &other);
  Vector3D rotate(const Vector3D &vec);
  static Vector3D rotate(const Vector3D &vec, const Vector3D &axis,
                         const double ang);

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

  double wl = lhs.w();
  double wr = rhs.w();

  Vector3D vl = lhs.vec();
  Vector3D vr = rhs.vec();

  return Quaternion(wl * wr - dot(vl, vr), vr * wl + vl * wr + cross(vl, vr));
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
  return lhs.w() == rhs.w() && lhs.vec() == rhs.vec();
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
inline Vector3D Quaternion::rotate(const Vector3D &vec, const Vector3D &axis,
                                   const double ang) {
  Quaternion rotQ{normalize(axis), ang};
  Quaternion vecQ{0, vec};
  Quaternion res = rotQ * vecQ * rotQ.inv();

  return res.vec();
}

/**
 * @brief Multiplies a quaternion in place
 *
 * @param other quaternion
 *
 * @returns Quaternion multiplied in place
 */
inline Quaternion &Quaternion::operator*=(const Quaternion &other) {
  Quaternion res = (*this) * other;
  m_w = res.w();
  m_vec = res.vec();

  return *this;
}

/**
 * @brief Rotates a vector with current quaternion object
 *
 * @param vec The vector to rotate
 *
 * @returns The rotated vector.
 */
inline Vector3D Quaternion::rotate(const Vector3D &vec) {
  Quaternion vecQ = {0, vec};
  Quaternion res = (*this) * vecQ * inv();

  return res.vec();
}
} // namespace imunano33

#endif
