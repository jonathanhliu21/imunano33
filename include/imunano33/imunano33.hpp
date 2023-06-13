/**
 * @file
 * @brief Contains the imunano33::IMUNano33 class
 */

#ifndef INCLUDE_IMUNANO33_IMUNANO33_HPP_
#define INCLUDE_IMUNANO33_IMUNANO33_HPP_

#include "imunano33/climate.hpp"
#include "imunano33/filter.hpp"
#include "imunano33/quaternion.hpp"
#include "imunano33/simplevectors.hpp"
#include "imunano33/unit.hpp"

namespace imunano33 {
using svector::Vector3D;

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
  IMUNano33();
  IMUNano33(const double favoring);
  IMUNano33(const double favoring, const Quaternion &initialQ);

  void updateClimate(const double temperature, const double humidity,
                     const double pressure);
  void updateIMU(const Vector3D &gyro, const Vector3D &accel,
                 const double deltaT);
  void update(const Vector3D &gyro, const Vector3D &accel, const double deltaT,
              const double temperature, const double humidity,
              const double pressure);
  void resetIMU();
  void zeroIMU();
  void resetClimate();
  void setRotQ(const Quaternion &rot);
  void setGyroFavoring(const double favoring);

  Quaternion getRotQ() const;
  double getGyroFavoring() const;
  template <TempUnit U> double getTemp();
  template <PressureUnit U> double getPressure();
  double getHumidity();

private:
  Quaternion m_initialQ;
  Filter m_filter;
  Climate m_climate;
};
} // namespace imunano33
#endif
