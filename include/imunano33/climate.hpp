/**
 * @file
 * @brief File containing the imunano33::Climate class
 */

#ifndef INCLUDE_IMUNANO33_CLIMATE_HPP_
#define INCLUDE_IMUNANO33_CLIMATE_HPP_

#include "imunano33/unit.hpp"

namespace imunano33 {
/**
 * @brief Handles climate data from Nano 33 (or other) sensors
 *
 * Objects of this class take in data from a temperature, humidity, and pressure
 * sensor (HTS221 for temperature and humidity and LPS22HB for pressure on the
 * Nano 33 BLE Sense), which can be converted to other units.
 *
 * This class is meant to be default initialized, then updated through the use
 * of the update() method.
 *
 * If a temperature sensor is not available, this can still be initialized, but
 * dataExists() will be false.
 */
class Climate {
public:
  /**
   * @brief Default constructor
   *
   * Initializes with no climate data. To update climate data, call update().
   */
  Climate() : m_dataExists{false}, m_temp{0}, m_humid{0}, m_pressure{0} {}

  /**
   * @brief Copy constructor
   */
  Climate(const Climate &other)
      : m_dataExists{other.m_dataExists}, m_temp{other.m_temp},
        m_humid{other.m_humid}, m_pressure{other.m_pressure} {}

  /**
   * @brief Assignment operator
   */
  Climate &operator=(const Climate &other) {
    if (this == &other) {
      return *this;
    }

    m_dataExists = other.m_dataExists;
    m_temp = other.m_temp;
    m_humid = other.m_humid;
    m_pressure = other.m_pressure;

    return *this;
  }

  /**
   * @brief Determines if climate data exists.
   *
   * This only returns false if initialized but update() has not been called
   * yet.
   *
   * @returns If data exists.
   */
  bool dataExists() const { return m_dataExists; }

  /**
   * @brief Gets temperature
   *
   * Check that this temperature measurement is valid with dataExists() first.
   *
   * Specify the unit in the template argument.
   *
   * @returns Temperature in given unit.
   */
  template <TempUnit U> double getTemp() const {
    double res;

    switch (U) {
    case FAHRENHEIT:
      res = m_temp * (9.0 / 5.0) + 32;
      break;
    case CELSIUS:
      res = m_temp;
      break;
    case KELVIN:
      res = m_temp + 273.15;
      break;
    default:
      res = m_temp;
    }

    return res;
  }

  /**
   * @brief Gets pressure
   *
   * Check that this pressure measurement is valid with dataExists() first.
   *
   * Specify the unit in the template argument.
   *
   * @returns Pressure in given unit.
   */
  template <PressureUnit U> double getPressure() const {
    double res;

    switch (U) {
    case KPA:
      res = m_pressure;
      break;
    case ATM:
      res = m_pressure * 0.00986923266716;
      break;
    case MMHG:
      res = m_pressure * 7.50062;
      break;
    case PSI:
      res = m_pressure * 0.1450377377;
      break;
    }

    return res;
  }

  /**
   * @brief Gets relative humidity
   *
   * Check that this humidity measurement is valid with dataExists() first.
   *
   * Unit is percent humidity.
   *
   * @returns Relative humidity
   */
  double getHumidity() const { return m_humid; }

  /**
   * @brief Updates climate data
   *
   * @param temp Temperature, in C
   * @param humid Relative humidity, in percent
   * @param pressure Pressure, in kPa
   */
  void update(const double temp, const double humid, const double pressure) {
    m_dataExists = true;
    m_temp = temp;
    m_humid = humid;
    m_pressure = pressure;
  }

  /**
   * @brief Resets climate data
   *
   * dataExists() will be false after this is called.
   */
  void reset() { m_dataExists = false; }

private:
  bool m_dataExists;

  double m_temp;
  double m_humid;
  double m_pressure;
};
} // namespace imunano33

#endif
