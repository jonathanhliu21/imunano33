/**
 * @file
 * @brief Contains the imunano33::IMUNano33 class
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
 */

#ifndef INCLUDE_IMUNANO33_IMUNANO33_HPP_
#define INCLUDE_IMUNANO33_IMUNANO33_HPP_

#include "imunano33/climate.hpp"
#include "imunano33/filter.hpp"
#include "imunano33/quaternion.hpp"
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
 * @brief A data processor for IMU and climate data from an Arduino Nano 33 BLE
 * Sense.
 *
 * This processor includes a 6-axis complementary filter which can determine the
 * orientation of the Arduino from its built-in IMU, the LSM9DS1, and the 3-axis
 * acceleration and 3-axis angular velocity measurements. The reason why the
 * magnetometer is not used is due to unknown magnetic interference which could
 * affect sensor measurements, which means that the yaw measurement will
 * inevitably drift over time. Note that the filter assumes that the gyro and
 * accelerometer are calibrated.
 *
 * The xyz axes are defined as following for the Nano 33 BLE sense (or any
 * Arduino Nano): With the Arduino flat on a table and the sensors facing up and
 * the opening of the Micro USB port facing towards the front, the positive x
 * direction points towards the front, the positive y direction points
 * perpendicular and to the left, and the positive z direction points directly
 * up. Note that the Arduino may not measure the angular velocities and
 * accelerations with respect to these axes, so you may need to correct the
 * measured values.
 *
 * The processor also takes in climate (temperature, humidity, and pressure)
 * from the HTS221 (for temperature and humidity) and the LPS22HB sensors (for
 * the pressure). These values should be measured in celsius (for temperature),
 * kilopascals (for pressure), and percent relative humidity. The temperature
 * and pressure units can later be converted to other units (see
 * imunano33::TempUnit and imunano33::PressureUnit for supported units).
 *
 * This can also be used outside the context of a Nano 33 BLE Sense (such as
 * with an external MPU-6050 IMU or a DHT22 temperature and humidity sensor). If
 * certain values are unknown, you can substitute zerores for these values, or
 * not call the corresponding update() method. For example, if you do not want
 * to have an accelerometer ccorrect gyro measurements, pass a zero vector for
 * the accelerometer measurement so that there will be no correction. If you do
 * not know climate data, do not call updateClimate() and only call updateIMU().
 */
class IMUNano33 {
public:
  /**
   * @brief Default constructor
   *
   * Sets filter gyro favoring to 0.98 and initial orientation to be pointing in
   * the positive x-direction.
   */
  IMUNano33() = default;

  /**
   * @brief Constructor
   *
   * Sets initial orientation to be pointing in the positive x-direction.
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
  IMUNano33(const num_t gyroFavoring) : m_filter{gyroFavoring} {}

  /**
   * @brief Constructor
   *
   * @param gyroFavoring Determines how much gravity should correct, in the
   * range [0, 1]. 0 means that gravity should fully correct the error (does not
   * mean that orientation is solely determined by gravity), and 1 means that
   * gravity does not correct error at all.
   * @param initialQ The initial rotation quaternion. When resetIMU() is called,
   * the orientation quaternion will be set to this value.
   *
   * @note If gyroFavoring is less than 0 or greater than 1, it gets clamped to
   * 0 or 1.
   * @note If initialQ is unnormalized, then the method will normalize it. If
   * initialQ is set to be zeroes, this will result in undefined behavior.
   * @note If favoring is too high (> 0.99), then there might be latency in
   * gravity correction.
   */
  IMUNano33(const num_t gyroFavoring, const Quaternion &initialQ)
      : m_initialQ{initialQ}, m_filter{gyroFavoring, initialQ} {}

  /**
   * @brief Copy constructor
   */
  IMUNano33(const IMUNano33 &other) = default;

  /**
   * @brief Assignment operator
   */
  IMUNano33 &operator=(const IMUNano33 &other) = default;

  /**
   * @brief Destructor
   */
  ~IMUNano33() = default;

  /**
   * @brief Move constructor
   */
  IMUNano33(IMUNano33 &&) = default;

  /**
   * @brief Move assignment
   */
  IMUNano33 &operator=(IMUNano33 &&) = default;

  /**
   * @brief Updates climate data
   *
   * It is important that you retrieve the data in the units listed below, or
   * the units will not be accurate.
   *
   * @param temperature Temperature, in C
   * @param humidity Relative humidity, in percent
   * @param pressure Pressure, in kPa
   */
  void updateClimate(const num_t temperature, const num_t humidity,
                     const num_t pressure) {
    m_climate.update(temperature, humidity, pressure);
  }

  /**
   * @brief Updates IMU data
   *
   * It is important that the gyroscope is given in radians per second, or the
   * orientation data will be inaccurate. The accelerometer measurement can be
   * in any unit, but m / s^2 is preferred as it is SI.
   *
   * @param accel Accelerometer reading, in <x, y, z>, where positive z is up
   * (important for gravity corrections), and xy is translational motion. The
   * note comes with more details specific to the Arduino Nano 33.
   * @param gyro Gyroscope reading (<roll, pitch, yaw> in rad/s)
   * @param deltaT The time between this measurement and the previous
   * measurement, in seconds. If this is the first measurement, deltaT would
   * refer to the time since startup (when initialQ was measured).
   */
  void updateIMU(const Vector3D &accel, const Vector3D &gyro,
                 const num_t deltaT) {
    m_filter.update(accel, gyro, deltaT);
  }

  /**
   * @brief Updates IMU acceleration data
   *
   * @param accel Accelerometer reading, in <x, y, z>, where positive z is up
   * (important for gravity corrections), and xy is translational motion. The
   * note comes with more details specific to the Arduino Nano 33.
   */
  void updateIMUAccel(const Vector3D &accel) { m_filter.updateAccel(accel); }

  /**
   * @brief Updates IMU gyroscope data.
   *
   * It is important that the gyroscope is given in radians per second, or the
   * orientation data will be inaccurate. The accelerometer measurement can be
   * in any unit, but m / s^2 is preferred as it is SI.
   *
   * @param gyro Gyroscope reading (<roll, pitch, yaw> in rad/s)
   * @param deltaT The time between this measurement and the previous
   * measurement, in seconds. If this is the first measurement, deltaT would
   * refer to the time since startup (when initialQ was measured).
   */
  void updateIMUGyro(const Vector3D &gyro, const num_t deltaT) {
    m_filter.updateGyro(gyro, deltaT);
  }

  /**
   * @brief Updates both IMU and climate data
   *
   * It is important that the gyroscope is given in radians per second, or the
   * orientation data will be inaccurate. The accelerometer measurement can be
   * in any unit, but m / s^2 is preferred as it is SI.
   *
   * It is important that you retrieve the data in the units listed below, or
   * the units will not be accurate.
   *
   * @param accel Accelerometer reading, in <x, y, z>, where positive z is up
   * (important for gravity corrections), and xy is translational motion. The
   * note comes with more details specific to the Arduino Nano 33.
   * @param gyro Gyroscope reading (<roll, pitch, yaw> in rad/s)
   * @param deltaT The time between this measurement and the previous
   * measurement, in seconds. If this is the first measurement, deltaT would
   * refer to the time since startup (when initialQ was measured).
   * @param temperature Temperature, in C
   * @param humidity Relative humidity, in percent
   * @param pressure Pressure, in kPa
   */
  void update(const Vector3D &accel, const Vector3D &gyro, const num_t deltaT,
              const num_t temperature, const num_t humidity,
              const num_t pressure) {
    updateIMU(accel, gyro, deltaT);
    updateClimate(temperature, humidity, pressure);
  }

  /**
   * @brief Resets IMU orientation to the initialQ argument provided in the
   * constructor.
   *
   * All measurements from this point on will be in the frame of reference of
   * the initial quaternion.
   */
  void resetIMU() { m_filter.setRotQ(m_initialQ); }

  /**
   * @brief Sets current IMU orientation to be facing the positive X-axis..
   *
   * All measurements from this point on will be relative to where you set the
   * orientation to be the positive X-axis.
   */
  void zeroIMU() { m_filter.reset(); }

  /**
   * @brief Resets climate data
   *
   * climateDataExists() will be false after this is called.
   */
  void resetClimate() { m_climate.reset(); }

  /**
   * @brief Sets rotation quaternion for the filter
   *
   * All measurements from this point on will be relative to this quaternion.
   *
   * @param q The rotation quaternion
   *
   * @note If q is unnormalized, then this method will normalize it. If q is set
   * to be zeros, this will result in undefined behavior.
   */
  void setRotQ(const Quaternion &q) { m_filter.setRotQ(q); }

  /**
   * @brief Sets gyro favoring
   *
   * @param favoring Determines how much gravity should correct, in the
   * range [0, 1]. 0 means that gravity should fully correct the error (does not
   * mean that orientation is solely determined by gravity), and 1 means that
   * gravity does not correct error at all.
   *
   * @note If favoring is less than 0 or greater than 1, it will be clamped to 0
   * or 1.
   */
  void setGyroFavoring(const num_t favoring) {
    m_filter.setGyroFavoring(favoring);
  }

  /**
   * @brief Gets rotation quaternion of the complementary filter
   *
   * @returns rotation quaternion
   */
  Quaternion getRotQ() const { return m_filter.getRotQ(); }

  /**
   * @brief Gets gyroscope favoring
   *
   * @returns gyro favoring
   */
  num_t getGyroFavoring() const { return m_filter.getGyroFavoring(); }

  /**
   * @brief Gets temperature
   *
   * Check that this temperature measurement is valid with climateDataExists()
   * first.
   *
   * @tparam U Temperature unit.
   *
   * @returns Temperature in given unit.
   */
  template <TempUnit U> num_t getTemperature() const {
    return m_climate.getTemp<U>();
  }

  /**
   * @brief Gets pressure
   *
   * Check that this pressure measurement is valid with climateDataExists()
   * first.
   *
   * @tparam U Pressure unit.
   *
   * @returns Pressure in given unit.
   */
  template <PressureUnit U> num_t getPressure() const {
    return m_climate.getPressure<U>();
  }

  /**
   * @brief Gets relative humidity
   *
   * Check that this humidity measurement is valid with climateDataExists()
   * first.
   *
   * Unit is percent humidity.
   *
   * @returns Relative humidity
   */
  num_t getHumidity() const { return m_climate.getHumidity(); }

  /**
   * @brief Determines if climate data exists.
   *
   * This only returns false if the object is initialized but none of update()
   * or updateClimate() have not been called yet.
   *
   * @returns If data exists.
   */
  bool climateDataExists() const { return m_climate.dataExists(); }

private:
  Quaternion m_initialQ;
  Filter m_filter;
  Climate m_climate;
};
} // namespace imunano33
#endif
