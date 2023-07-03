/**
 * @file
 * @brief File containing the imunano33::MathUtil class
 */

#ifndef INCLUDE_IMUNANO33_MATHUTIL_HPP_
#define INCLUDE_IMUNANO33_MATHUTIL_HPP_

#include <cmath>
#include <cstddef>

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
using svector::Vector3D;
#endif

/**
 * @brief Utility static methods for math calculations
 */
class MathUtil {
public:
  /**
   * @brief Determines if number is near zero with given precision.
   *
   * The tolerance is 0.00001.
   *
   * @param num The number to determine if near zero
   *
   * @returns if the num is near zero
   */
  static bool nearZero(const num_t num) { return nearZero(num, NEAR_ZERO); }

  /**
   * @brief Determines if number is near zero with given precision.
   *
   * @param num The number to determine if near zero
   * @param tol Tolerance within zero such that a number strictly less than this
   * tolerance is counted as zero.
   *
   * @returns if the num is near zero
   */
  static bool nearZero(const num_t num, const num_t tol) {
    return std::abs(num) < tol;
  }

  /**
   * @brief Determines if vector is near zero with given precision.
   *
   * The tolerance is 0.00001.
   *
   * @param vec The vector to determine if near zero
   *
   * @returns if the vector is near zero
   */
  static bool nearZero(const Vector3D &vec) { return nearZero(vec, NEAR_ZERO); }

  /**
   * @brief Determines if vector is near zero with given precision.
   *
   * @param vec The vector to determine if near zero
   * @param tol Tolerance within zero such that a number strictly less than this
   * tolerance is counted as zero.
   *
   * @returns if the vector is near zero
   */
  static bool nearZero(const Vector3D &vec, const num_t tol) {
#ifdef IMUNANO33_EMBED
    return vec.x < tol && vec.y < tol && vec.z < tol;
#else
    return std::none_of(vec.begin(), vec.end(),
                        [tol](const num_t &el) { return std::abs(el) >= tol; });
#endif
  }

  /**
   * @brief Determines if num1 is nearly equal to num2
   *
   * The tolerance is 0.00001.
   *
   * This is helpful for comparing the equality of floating point numbers.
   *
   * @param num1 A number to compare
   * @param num2 A number to compare
   *
   * @returns If the numbers are near each other such that they can be counted
   * as equal.
   */
  static bool nearEq(const num_t num1, const num_t num2) {
    return nearEq(num1, num2, NEAR_ZERO);
  }

  /**
   * @brief Determines if num1 is nearly equal to num2
   *
   * This is helpful for comparing the equality of floating point numbers.
   *
   * @param num1 A number to compare
   * @param num2 A number to compare
   * @param tol Tolerance within zero such that a number strictly less than this
   * tolerance is counted as zero.
   *
   * @returns If the numbers are near each other such that they can be counted
   * as equal.
   */
  static bool nearEq(const num_t num1, const num_t num2, const num_t tol) {
    return std::abs(num1 - num2) < tol;
  }

  /**
   * @brief Restricts num between lo and hi
   *
   * If num < lo, returns lo, if num > hi, returns hi, otherwise returns num. If
   * lo > hi, then behavior is undefined.
   *
   * @param num Number to clamp
   * @param lo Lower bound
   * @param hi Upper bound
   *
   * @returns Clamped number
   */
  template <typename T> static T clamp(const T &num, const T &lo, const T &hi) {
    return num < lo ? lo : num > hi ? hi : num;
  }

private:
  static constexpr num_t NEAR_ZERO = 0.00001;
};

} // namespace imunano33

#endif
