/**
 * @file
 * @brief File containing the imunano33::Filter class
 */

#ifndef INCLUDE_IMUNANO33_FILTER_HPP_
#define INCLUDE_IMUNANO33_FILTER_HPP_

#include "imunano33/mathutil.hpp"
#include "imunano33/quaternion.hpp"
#include "imunano33/simplevectors.hpp"

namespace imunano33 {
using svector::Vector3D;

/**
 * @brief A complementary filter for a 6 axis IMU using quaternions.
 *
 * Integrates the gyro measurements, then uses a small fraction of the gravity
 * measurements (from the accelerometer) to correct the orientation given from
 * the gyro measurements. The fraction is specified through the gyroFavoring
 * parameter in the constructor.
 *
 * The math and details are based on these lectures from Stanford:
 * * https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf
 * * https://stanford.edu/class/ee267/lectures/lecture10.pdf
 */
class Filter {
public:
  /**
   * @brief Default Constructor
   *
   * Initializes intiial quaternion to [1, 0, 0, 0] (or facing towards +x
   * direction) and gyro favoring to 0.98. See other constructors for more
   * information about gyro favoring.
   */
  Filter() : m_gyroFavoring{0.98}, m_qRot{1, Vector3D{}} {}

  /**
   * @brief Constructor
   *
   * @param gyroFavoring Determines how much gravity should correct, in the
   * range [0, 1]. 0 means that gravity should fully correct the error (does not
   * mean that orientation is solely determined by gravity), and 1 means that
   * gravity does not correct error at all.
   *
   * @note If gyroFavoring is less than 0 or greater than 1, it gets clamped to
   * 0 or 1.
   * @note If favoring is too high (> 0.99), then there might be latency in
   * gravity correction.
   */
  Filter(const double gyroFavoring) : m_qRot{1, Vector3D{}} {
    m_gyroFavoring = MathUtil::clamp(gyroFavoring, 0.0, 1.0);
  }

  /**
   * @brief Constructor
   *
   * @param gyroFavoring Determines how much gravity should correct, in the
   * range [0, 1]. 0 means that gravity should fully correct the error (does not
   * mean that orientation is solely determined by gravity), and 1 means that
   * gravity does not correct error at all.
   * @param initialQ The initial rotation quaternion.
   *
   * @note If gyroFavoring is less than 0 or greater than 1, it gets clamped to
   * 0 or 1.
   * @note If initialQ is unnormalized, then the method will normalize it. If
   * initialQ is set to be zeroes, this will result in undefined behavior.
   * @note If favoring is too high (> 0.99), then there might be latency in
   * gravity correction.
   */
  Filter(const double gyroFavoring, const Quaternion &initialQ)
      : m_qRot{initialQ.unit()} {
    m_gyroFavoring = MathUtil::clamp(gyroFavoring, 0.0, 1.0);
  }

  /**
   * @brief Copy constructor
   */
  Filter(const Filter &other) = default;

  /**
   * @brief Assignment operator
   */
  Filter &operator=(const Filter &other) = default;

  /**
   * @brief Destructor
   */
  ~Filter() = default;

  /**
   * @brief Move constructor
   */
  Filter(Filter &&) = default;

  /**
   * @brief Move assignment
   */
  Filter &operator=(Filter &&) = default;

  /**
   * @brief Updates filter
   *
   * If you plan on only using the gyroscope measurements, then pass in
   * a zero vector for the acceleration, as accelerometer corrections will not
   * be performed if the acceleration vector is zero.
   *
   * @param accel Accelerometer reading, in <x, y, z>, where positive z is up
   * (important for gravity corrections), and xy is translational motion. The
   * note comes with more details specific to the Arduino Nano 33.
   * @param gyro Gyroscope reading (<roll, pitch, yaw> in rad/s)
   * @param time The time it took for the reading to happen (in s)
   * @param favoring Determines how much gravity should correct, in the range
   * [0, 1]. 0 means that gravity should fully correct the error (does not mean
   * that orientation is solely determined by gravity), and 1 means that gravity
   * does not correct error at all.
   *
   * @note With the opening of the USB port facing front and the Arduino's
   * sensors facing up, the positive x axis is to the front, the positive y axis
   * is to the left, and the positive z axis is to the top.
   */
  void update(const Vector3D &accel, const Vector3D &gyro, const double time,
              const double favoring) {
    // math from:
    // https://stanford.edu/class/ee267/lectures/lecture10.pdf
    // https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf

    // rotation quaternion from gyroscope readings
    Quaternion qGyroCur;
    if (MathUtil::nearZero(gyro)) {
      // if gyro reading is 0, take current quaternion to be corrected
      qGyroCur = m_qRot;
    } else {
      // otherwise integrate quaternion reading
      const Quaternion qGyroDelta{normalize(gyro), time * magn(gyro)};
      qGyroCur = m_qRot * qGyroDelta;
    }

    // don't bother with acceleration correction if acceleration is basically
    // 0
    if (MathUtil::nearZero(accel)) {
      m_qRot = qGyroCur;
      return;
    }

    // gravity vector rotation
    const Quaternion qAccelBody{0, accel};
    const Quaternion qAccelWorld =
        qGyroCur * qAccelBody *
        qGyroCur.conj(); // rotates body acceleration by gyro measurements

    // correcting gyro drift with accelerometer
    const Vector3D vecAccelWorldNorm = normalize(qAccelWorld.vec());
    const Vector3D vecAccelGravity{0, 0, -1};
    const Vector3D vecRotAxis =
        cross(vecAccelWorldNorm,
              vecAccelGravity); // rotation axis for correction rotation from
                                // estimated gravity vector (from gyro
                                // readings) to true gravity vector
    const double rotAngle = std::acos(
        MathUtil::clamp(dot(vecAccelGravity, vecAccelWorldNorm) /
                            (magn(vecAccelGravity) * magn(vecAccelWorldNorm)),
                        -1.0,
                        1.0)); // angle to rotate to correct acceleration vector

    // if angle needed to rotate is 0 or the axis to rotate around is 0, then
    // don't bother correcting
    if (MathUtil::nearZero(rotAngle) || MathUtil::nearZero(vecRotAxis)) {
      m_qRot = qGyroCur;
      return;
    }

    // complementary filter
    const Quaternion qAccelCur{normalize(vecRotAxis),
                               (1 - favoring) * rotAngle};
    m_qRot = qAccelCur * qGyroCur;
  }

  /**
   * @brief Updates filter
   *
   * If you plan on only using the gyroscope measurements, then pass in
   * a zero vector for the acceleration, as accelerometer corrections will not
   * be performed if the acceleration vector is zero.
   *
   * @param accel Accelerometer reading, in <x, y, z>, where positive z is up
   * (important for gravity corrections), and xy is translational motion. The
   * note comes with more details specific to the Arduino Nano 33.
   * @param gyro Gyroscope reading (in rad/s)
   * @param time The time it took for the reading to happen (in s)
   *
   * @note With the opening of the USB port facing front and the Arduino's
   * sensors facing up, the positive x axis is to the front, the positive y axis
   * is to the left, and the positive z axis is to the top.
   */
  void update(const Vector3D &accel, const Vector3D &gyro, const double time) {
    update(accel, gyro, time, m_gyroFavoring);
  }

  /**
   * @brief Resets quaternion to [1, 0, 0, 0], or facing towards position x
   * direction.
   */
  void reset() { setRotQ({1, {0, 0, 0}}); }

  /**
   * @brief Gets rotation quaternion of the complementary filter
   *
   * @returns rotation quaternion
   */
  Quaternion getRotQ() const { return m_qRot; }

  /**
   * @brief Gets gyroscope favoring
   *
   * @returns gyro favoring
   */
  double getGyroFavoring() const { return m_gyroFavoring; }

  /**
   * @brief Sets rotation quaternion for the filter
   *
   * @param q The rotation quaternion
   *
   * @note If q is unnormalized, then this method will normalize it. If q is set
   * to be zeros, this will result in undefined behavior.
   */
  void setRotQ(const Quaternion &q) { m_qRot = q.unit(); }

  /**
   * @brief Sets gyro favoring
   *
   * @param favoring The new gyro favoring, in the range [0, 1]
   *
   * @note If favoring is less than 0 or greater than 1, it will be clamped to 0
   * or 1.
   */
  void setGyroFavoring(const double favoring) {
    m_gyroFavoring = MathUtil::clamp(favoring, 0.0, 1.0);
  }

private:
  double m_gyroFavoring;

  Quaternion m_qRot;
};

} // namespace imunano33

#endif
