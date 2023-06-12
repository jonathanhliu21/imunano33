#ifndef INCLUDE_IMUNANO33_MATHUTIL_HPP_
#define INCLUDE_IMUNANO33_MATHUTIL_HPP_

#include <cmath>

#include "imunano33/simplevectors.hpp"

namespace imunano33 {
using svector::Vector3D;

/**
 * @brief Utility static methods for math calculations
 */
class MathUtil {
public:
  /**
   * @brief Determines if double is near zero with given precision.
   *
   * @param num The number to determine if near zero
   * @param tol Tolerance within zero such that a number strictly less than this
   * tolerance is counted as zero.
   *
   * @returns if the num is near zero
   */
  static bool nearZero(const double num, const double tol = NEAR_ZERO) {
    return std::abs(num) < tol;
  }

  /**
   * @brief Determines if vector is near zero with given precision.
   *
   * @param vec The vector to determine if near zero
   * @param tol Tolerance within zero such that a number strictly less than this
   * tolerance is counted as zero.
   *
   * @returns if the vector is near zero
   */
  static bool nearZero(const Vector3D &vec, const double tol = NEAR_ZERO) {
    for (const double &i : vec) {
      if (std::abs(i) >= tol) {
        return false;
      }
    }
    return true;
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
  static bool nearEq(const double num1, const double num2,
                     const double tol = NEAR_ZERO) {
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
  static constexpr double NEAR_ZERO = 0.00001;
};

} // namespace imunano33

#endif
