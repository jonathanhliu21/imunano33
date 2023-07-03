/**
 * @file
 *
 * @brief File containing the imunano33::Quaternion class
 */

#ifndef INCLUDE_IMUNANO33_QUATERNION_HPP_
#define INCLUDE_IMUNANO33_QUATERNION_HPP_

#ifdef IMUNANO33_EMBED
#include <math.h>
#else
#include <cmath>
#endif

#ifdef IMUNANO33_EMBED
#include "imunano33/sv_embed.hpp"
#else
#include "imunano33/simplevectors.hpp"
#endif
#include "imunano33/unit.hpp"

namespace imunano33 {
#ifdef IMUNANO33_EMBED
using Vector3D =
    svector::EmbVec3D; //!< Alias to vector type in embedded systems
#else
using std::cos;
using std::sin;
using std::sqrt;
using svector::Vector3D;
#endif

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
  Quaternion(const num_t w, const Vector3D &vec) : m_vec{vec} {
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
  Quaternion(const Vector3D &vec, const num_t ang) : m_w{cos(ang / 2)} {
    const Vector3D norm = normalize(vec);
    m_vec = normalize(vec) * sin(ang / 2);
  }

  /**
   * @brief Copy constructor
   *
   * @param other Other quaternion
   */
  Quaternion(const Quaternion &other) = default;

  /**
   * @brief Assignment operator
   */
  Quaternion &operator=(const Quaternion &other) = default;

  /**
   * @brief Destructor
   */
  ~Quaternion() = default;

  /**
   * @brief Move constructor
   */
  Quaternion(Quaternion &&) = default;

  /**
   * @brief Move assignment
   */
  Quaternion &operator=(Quaternion &&) = default;

  /**
   * @brief Gets the scalar component of the quaternion
   *
   * @returns The scalar component
   */
  num_t w() const { return m_w; }

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
    const Quaternion conju = conj();
    const num_t mag = norm();
    const num_t newW = conju.w() / (mag * mag);
    const Vector3D newVec = conju.vec() / (mag * mag);

    return Quaternion{newW, newVec};
  }

  /**
   * @brief Gets quaternion norm
   *
   * This behaves the same as a "magnitude" in 4-dimensional vector terms.
   *
   * @returns Quaternion norm
   */
  num_t norm() const {
    return sqrt(m_w * m_w + x(m_vec) * x(m_vec) + y(m_vec) * y(m_vec) +
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
    const num_t mag = norm();
    const num_t newW = m_w / mag;
    const Vector3D newVec = m_vec / mag;

    return Quaternion{newW, newVec};
  }

  // defined later, where operators are defined
  Quaternion &operator*=(const Quaternion &other);
  Vector3D rotate(const Vector3D &vec) const;
  static Vector3D rotate(const Vector3D &vec, const Vector3D &axis, num_t ang);

private:
  num_t m_w;
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
#ifdef IMUNANO33_EMBED
  using Vector3D =
      svector::EmbVec3D; //!< Alias to vector type in embedded systems
#else
  using svector::Vector3D;
#endif

  const num_t wl = lhs.w();
  const num_t wr = rhs.w();

  const Vector3D vl = lhs.vec();
  const Vector3D vr = rhs.vec();

  return Quaternion{wl * wr - dot(vl, vr), vr * wl + vl * wr + cross(vl, vr)};
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
                                   const num_t ang) {
  const Quaternion rotQ{normalize(axis), ang};
  const Quaternion vecQ{0, vec};
  const Quaternion res = rotQ * vecQ * rotQ.conj();

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
  const Quaternion res = (*this) * other;
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
inline Vector3D Quaternion::rotate(const Vector3D &vec) const {
  const Quaternion vecQ = {0, vec};
  const Quaternion res = (*this) * vecQ * inv();

  return res.vec();
}
} // namespace imunano33

#endif
