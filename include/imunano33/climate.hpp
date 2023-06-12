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
  Climate();

  /**
   * @brief Determines if climate data exists.
   *
   * This only returns false if initialized but update() has not been called
   * yet.
   *
   * @returns If data exists.
   */
  bool dataExists() const;
  template <TempUnit U> double getTemp() const;
  template <PressureUnit U> double getPressure() const;
  double getHumidity() const;

  void update(const double temp, const double humid, const double pressure);
  void reset();

private:
  bool m_dataExists;

  double m_temp;
  double m_humid;
  double m_pressure;
};
} // namespace imunano33

#endif
