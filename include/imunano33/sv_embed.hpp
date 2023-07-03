/**
 * @file embed.h
 *
 * @brief A minimized version of vectors for embedded devices without access to
 * the STL (such as on an Arduino, hence the ".h" instead of ".hpp").
 *
 * This file is meant to be a standalone file, so it is not included in
 * vectors.hpp.
 *
 * @internal
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Jonathan Liu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * @endinternal
 *
 * @copyright Copyright (c) 2023 Jonathan Liu. This project is released under
 * the MIT License. All rights reserved.
 */

#ifndef INCLUDE_SVECTOR_EMBED_HPP_
#define INCLUDE_SVECTOR_EMBED_HPP_

#include <math.h> // acosf, atan2f, cosf, sinf, sqrtf

namespace svector {
/**
 * @brief A minimal 2D vector representation.
 *
 * @note Uses floats to store data types rather than doubles in order to save
 * memory, which means that the vector type is not as precise.
 */
struct EmbVec2D {
  /**
   * @brief No-argument constructor.
   *
   * Initializes a zero vector.
   */
  EmbVec2D() : x{0}, y{0} {};

  /**
   * @brief Initializes a vector given xy components.
   *
   * @param xOther The x-component.
   * @param yOther The y-component.
   */
  EmbVec2D(const float xOther, const float yOther) : x{xOther}, y{yOther} {}

  /**
   * @brief Copy constructor.
   */
  EmbVec2D(const EmbVec2D &other) = default;

  /**
   * @brief Move constructor.
   *
   * Uses C++ default move constructor.
   */
  EmbVec2D(EmbVec2D &&) noexcept = default;

  /**
   * @brief Assignment operator.
   */
  EmbVec2D &operator=(const EmbVec2D &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    x = other.x;
    y = other.y;
    return *this;
  }

  /**
   * @brief Move assignment operator
   *
   * Uses C++ default move assignment operator.
   */
  EmbVec2D &operator=(EmbVec2D &&) noexcept = default;

  /**
   * @brief Destructor
   *
   * Uses C++ default destructor.
   */
  virtual ~EmbVec2D() = default;

  /**
   * @brief In-place addtion
   *
   * Adds another vector object to current vector.
   *
   * @param other The other object to add.
   */
  EmbVec2D &operator+=(const EmbVec2D &other) {
    x += other.x;
    y += other.y;
    return *this;
  }

  /**
   * @brief In-place subtraction
   *
   * Subtracts another vector object from current vector.
   *
   * @param other The other object to subtract.
   */
  EmbVec2D &operator-=(const EmbVec2D &other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  /**
   * @brief In-place scalar multiplication
   *
   * Performs scalar multiplication on the current object.
   *
   * @param other The number to multiply by.
   */
  EmbVec2D &operator*=(const float &other) {
    x *= other;
    y *= other;
    return *this;
  }

  /**
   * @brief In-place scalar division
   *
   * Performs scalar division on the current object.
   *
   * @param other The number to divide by.
   */
  EmbVec2D &operator/=(const float &other) {
    x /= other;
    y /= other;
    return *this;
  }

  float x; //!< The x-component of the 2D vector.
  float y; //!< The y-component of the 2D vector.
};

/**
 * @brief A minimal 3D vector representation.
 *
 * @note Uses floats to store data types rather than doubles in order to save
 * memory, which means that the vector type is not as precise.
 */
struct EmbVec3D {
  /**
   * @brief No-argument constructor.
   *
   * Initializes a zero vector.
   */
  EmbVec3D() : x{0}, y{0}, z{0} {}

  /**
   * @brief Initializes a vector given xyz components.
   *
   * @param xOther The x-component.
   * @param yOther The y-component.
   * @param zOther The z-component.
   */
  EmbVec3D(const float xOther, const float yOther, const float zOther)
      : x{xOther}, y{yOther}, z{zOther} {}

  /**
   * @brief Copy constructor.
   */
  EmbVec3D(const EmbVec3D &other) = default;

  /**
   * @brief Move constructor
   *
   * Uses C++ default move constructor.
   */
  EmbVec3D(EmbVec3D &&) noexcept = default;

  /**
   * @brief Assignment operator.
   */
  EmbVec3D &operator=(const EmbVec3D &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }

  /**
   * @brief Move assignment operator
   *
   * Uses C++ default move assignment operator.
   */
  EmbVec3D &operator=(EmbVec3D &&) noexcept = default;

  /**
   * @brief Destructor
   *
   * Uses C++ default destructor.
   */
  virtual ~EmbVec3D() = default;

  /**
   * @brief In-place addtion
   *
   * Adds another vector object to current vector.
   *
   * @param other The other object to add.
   */
  EmbVec3D &operator+=(const EmbVec3D &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /**
   * @brief In-place subtraction
   *
   * Subtracts another vector object from current vector.
   *
   * @param other The other object to subtract.
   */
  EmbVec3D &operator-=(const EmbVec3D &other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /**
   * @brief In-place scalar multiplication
   *
   * Performs scalar multiplication on the current object.
   *
   * @param other The number to multiply by.
   */
  EmbVec3D &operator*=(const float &other) {
    x *= other;
    y *= other;
    z *= other;
    return *this;
  }

  /**
   * @brief In-place scalar division
   *
   * Performs scalar division on the current object.
   *
   * @param other The number to divide by.
   */
  EmbVec3D &operator/=(const float &other) {
    x /= other;
    y /= other;
    z /= other;
    return *this;
  }

  float x; //!< The x-component of the 3D vector.
  float y; //!< The y-component of the 3D vector.
  float z; //!< The z-component of the 3D vector.
};

/**
 * @brief Gets the x-component of a 2D vector.
 *
 * @param v A 2D Vector.
 *
 * @returns x-component of the vector.
 */
inline float x(const EmbVec2D &v) { return v.x; }

/**
 * @brief Sets the x-component of a 2D vector.
 *
 * @param v A 2D Vector.
 * @param xValue The x-value to set to the vector.
 */
inline void x(EmbVec2D &v, const float xValue) { v.x = xValue; }

/**
 * @brief Gets the x-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns x-component of the vector.
 */
inline float x(const EmbVec3D &v) { return v.x; }

/**
 * @brief Sets the x-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param xValue The x-value to set to the vector.
 */
inline void x(EmbVec3D &v, const float xValue) { v.x = xValue; }

/**
 * @brief Gets the y-component of a 2D vector.
 *
 * @param v A 2D Vector.
 *
 * @returns y-component of the vector.
 */
inline float y(const EmbVec2D &v) { return v.y; }

/**
 * @brief Sets the y-component of a 2D vector.
 *
 * @param v A 2D Vector.
 * @param yValue The y-value to set to the vector.
 */
inline void y(EmbVec2D &v, const float yValue) { v.y = yValue; }

/**
 * @brief Gets the y-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns y-component of the vector.
 */
inline float y(const EmbVec3D &v) { return v.y; }

/**
 * @brief Sets the y-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param yValue The y value to set to the vector.
 */
inline void y(EmbVec3D &v, const float yValue) { v.y = yValue; }

/**
 * @brief Gets the z-component of a 3D vector.
 *
 * @param v A 3D Vector.
 *
 * @returns z-component of the vector.
 */
inline float z(const EmbVec3D &v) { return v.z; }

/**
 * @brief Sets the z-component of a 3D vector.
 *
 * @param v A 3D Vector.
 * @param zValue The z value to set to the vector.
 */
inline void z(EmbVec3D &v, const float zValue) { v.z = zValue; }

/**
 * @brief Vector addition
 *
 * Performs vector addition and returns a new vector representing the sum of
 * the two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
inline EmbVec2D operator+(const EmbVec2D &lhs, const EmbVec2D &rhs) {
  return EmbVec2D{lhs.x + rhs.x, lhs.y + rhs.y};
}

/**
 * @brief Vector subtraction
 *
 * Performs vector subtraction and returns a new vector representing the
 * difference of the two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
inline EmbVec2D operator-(const EmbVec2D &lhs, const EmbVec2D &rhs) {
  return EmbVec2D{lhs.x - rhs.x, lhs.y - rhs.y};
}

/**
 * @brief Negative of a vector
 *
 * Makes all components of a vector negative.
 *
 * This can also be thought of flipping the direction of the vector.
 *
 * @param vec A 2D vector.
 *
 * @returns A new vector representing the negative of the given vector.
 */
inline EmbVec2D operator-(const EmbVec2D &vec) {
  return EmbVec2D{-vec.x, -vec.y};
}

/**
 * @brief Positive of a vector
 *
 * Represents value of vector, with the unary plus operator applied to each
 * component. In almost all cases, this should result in the original vector.
 *
 * @param vec A 2D vector.
 *
 * @returns The same 2D vector.
 */
inline EmbVec2D operator+(const EmbVec2D &vec) {
  return EmbVec2D{+vec.x, +vec.y};
}

/**
 * @brief Scalar multiplication
 *
 * Performs scalar multiplication and returns a new vector representing the
 * product.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
inline EmbVec2D operator*(const EmbVec2D &lhs, const float rhs) {
  return EmbVec2D{lhs.x * rhs, lhs.y * rhs};
}

/**
 * @brief Scalar division
 *
 * Performs scalar division and returns a new vector representing the
 * quotient.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
inline EmbVec2D operator/(const EmbVec2D &lhs, const float rhs) {
  return EmbVec2D{lhs.x / rhs, lhs.y / rhs};
}

/**
 * @brief Compares equality of two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors compare equal.
 */
inline bool operator==(const EmbVec2D &lhs, const EmbVec2D &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

/**
 * @brief Compares inequality of two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors do not compare equal.
 */
inline bool operator!=(const EmbVec2D &lhs, const EmbVec2D &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Calculates the dot product of two vectors.
 *
 * @param lhs First vector.
 * @param rhs Second vector.
 *
 * @returns The dot product of lhs and rhs.
 */
inline float dot(const EmbVec2D &lhs, const EmbVec2D &rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

/**
 * @brief Gets the magnitude of the vector.
 *
 * @param vec A 2D vector.
 *
 * @returns magnitude of vector.
 */
inline float magn(const EmbVec2D &vec) {
  return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

/**
 * @brief Gets the angle of a 2D vector in radians.
 *
 * The angle will be in the range (-π, π].
 *
 * @param vec A 2D vector.
 *
 * @returns angle of the vector.
 */
inline float angle(const EmbVec2D &vec) { return atan2f(vec.y, vec.x); }

/**
 * @brief Normalizes a vector.
 *
 * Finds the unit vector with the same direction angle as the current vector.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param vec A 2D vector.
 *
 * @returns Normalized vector.
 */
inline EmbVec2D normalize(const EmbVec2D &vec) { return vec / magn(vec); }

/**
 * @brief Determines whether a vector is a zero vector.
 *
 * @returns Whether the given vector is a zero vector.
 */
inline bool isZero(const EmbVec2D &vec) { return magn(vec) == 0; }

/**
 * @brief Rotates vector by a certain angle.
 *
 * The angle should be given in radians. The vector rotates
 * counterclockwise when the angle is positive and clockwise
 * when the angle is negative.
 *
 * @param vec A 2D vector.
 * @param ang the angle to rotate the vector, in radians.
 *
 * @returns a new, rotated vector.
 */
inline EmbVec2D rotate(const EmbVec2D &vec, const float ang) {
  //
  // Rotation matrix:
  //
  // | cos(ang)   -sin(ang) | |x|
  // | sin(ang)    cos(ang) | |y|
  //

  const auto xPrime = vec.x * cosf(ang) - vec.y * sinf(ang);
  const auto yPrime = vec.x * sinf(ang) + vec.y * cosf(ang);

  return EmbVec2D{xPrime, yPrime};
}

/**
 * @brief Vector addition
 *
 * Performs vector addition and returns a new vector representing the sum of
 * the two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
inline EmbVec3D operator+(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  return EmbVec3D{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

/**
 * @brief Vector subtraction
 *
 * Performs vector subtraction and returns a new vector representing the
 * difference of the two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the vector sum.
 */
inline EmbVec3D operator-(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  return EmbVec3D{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

/**
 * @brief Negative of a vector
 *
 * Makes all components of a vector negative of what they currently are.
 *
 * This can also be thought of flipping the direction of the vector.
 *
 * @param vec A 3D vector.
 *
 * @returns A new vector representing the negative of the given vector.
 */
inline EmbVec3D operator-(const EmbVec3D &vec) {
  return EmbVec3D{-vec.x, -vec.y, -vec.z};
}

/**
 * @brief Positive of a vector
 *
 * Represents value of vector, with the unary plus operator applied to each
 * component. In almost all cases, this should result in the original vector.
 *
 * @param vec A 3D vector.
 *
 * @returns The same 3D vector.
 */
inline EmbVec3D operator+(const EmbVec3D &vec) {
  return EmbVec3D{+vec.x, +vec.y, +vec.z};
}

/**
 * @brief Scalar multiplication
 *
 * Performs scalar multiplication and returns a new vector representing the
 * product.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
inline EmbVec3D operator*(const EmbVec3D &lhs, const float rhs) {
  return EmbVec3D{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

/**
 * @brief Scalar division
 *
 * Performs scalar division and returns a new vector representing the
 * quotient.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A new vector representing the scalar product.
 */
inline EmbVec3D operator/(const EmbVec3D &lhs, const float rhs) {
  return EmbVec3D{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
}

/**
 * @brief Compares equality of two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors compare equal.
 */
inline bool operator==(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

/**
 * @brief Compares inequality of two vectors.
 *
 * @param lhs The first vector.
 * @param rhs The second vector.
 *
 * @returns A boolean representing whether the two vectors do not compare equal.
 */
inline bool operator!=(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  return !(lhs == rhs);
}

/**
 * @brief Calculates the dot product of two vectors.
 *
 * @param lhs First vector.
 * @param rhs Second vector.
 *
 * @returns The dot product of lhs and rhs.
 */
inline float dot(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

/**
 * @brief Cross product of two vectors.
 *
 * @param lhs A 3D vector.
 * @param rhs A 3D vector.
 *
 * @returns The crossed 3D vector.
 */
inline EmbVec3D cross(const EmbVec3D &lhs, const EmbVec3D &rhs) {
  const float newx = lhs.y * rhs.z - lhs.z * rhs.y;
  const float newy = lhs.z * rhs.x - lhs.x * rhs.z;
  const float newz = lhs.x * rhs.y - lhs.y * rhs.x;

  return EmbVec3D{newx, newy, newz};
}

/**
 * @brief Gets the magnitude of the vector.
 *
 * @param vec A 3D vector.
 *
 * @returns magnitude of vector
 */
inline float magn(const EmbVec3D &vec) {
  return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

/**
 * @brief Normalizes a vector.
 *
 * Finds the unit vector with the same direction angle as the current vector.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param vec A 3D vector.
 *
 * @returns Normalized vector.
 */
inline EmbVec3D normalize(const EmbVec3D &vec) { return vec / magn(vec); }

/**
 * @brief Determines whether a vector is a zero vector.
 *
 * @returns Whether the given vector is a zero vector.
 */
inline bool isZero(const EmbVec3D &vec) { return magn(vec) == 0; }

/**
 * @brief Gets α angle.
 *
 * α is the angle between the vector and the x-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param vec A 3D Vector.
 *
 * @returns α.
 */
inline float alpha(const EmbVec3D &vec) { return acosf(vec.x / magn(vec)); }

/**
 * @brief Gets β angle.
 *
 * β is the angle between the vector and the y-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param vec A 3D Vector.
 *
 * @returns β.
 */
inline float beta(const EmbVec3D &vec) { return acosf(vec.y / magn(vec)); }

/**
 * @brief Gets γ angle.
 *
 * γ is the angle between the vector and the z-axis.
 *
 * @note This method will result in undefined behavior if the vector is a zero
 * vector (if the magnitude equals zero).
 *
 * @param vec A 3D Vector.
 *
 * @returns γ.
 */
inline float gamma(const EmbVec3D &vec) { return acosf(vec.z / magn(vec)); }

/**
 * @brief Rotates around x-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param vec A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline EmbVec3D rotateAlpha(const EmbVec3D &vec, const float ang) {
  //
  // Rotation matrix:
  //
  // |1   0           0     | |x|
  // |0  cos(ang)  −sin(ang)| |y|
  // |0  sin(ang)   cos(ang)| |z|
  //

  const auto xPrime = vec.x;
  const auto yPrime = vec.y * cosf(ang) - vec.z * sinf(ang);
  const auto zPrime = vec.y * sinf(ang) + vec.z * cosf(ang);

  return EmbVec3D{xPrime, yPrime, zPrime};
}

/**
 * @brief Rotates around y-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param vec A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline EmbVec3D rotateBeta(const EmbVec3D &vec, const float ang) {
  //
  // Rotation matrix:
  //
  // | cos(ang)  0  sin(ang)| |x|
  // |   0       1      0   | |y|
  // |−sin(ang)  0  cos(ang)| |z|
  //

  const auto xPrime = vec.x * cosf(ang) + vec.z * sinf(ang);
  const auto yPrime = vec.y;
  const auto zPrime = -vec.x * sinf(ang) + vec.z * cosf(ang);

  return EmbVec3D{xPrime, yPrime, zPrime};
}

/**
 * @brief Rotates around z-axis.
 *
 * Uses the basic gimbal-like 3D rotation matrices for rotation.
 *
 * @param vec A 3D vector.
 * @param ang The angle to rotate the vector, in radians.
 *
 * @returns A new, rotated vector.
 */
inline EmbVec3D rotateGamma(const EmbVec3D &vec, const float ang) {
  //
  // Rotation matrix:
  //
  // |cos(ang)  −sin(ang)  0| |x|
  // |sin(ang)  cos(ang)   0| |y|
  // |  0         0        1| |z|
  //

  const auto xPrime = vec.x * cosf(ang) - vec.y * sinf(ang);
  const auto yPrime = vec.x * sinf(ang) + vec.y * cosf(ang);
  const auto zPrime = vec.z;

  return EmbVec3D{xPrime, yPrime, zPrime};
}
} // namespace svector

#endif
