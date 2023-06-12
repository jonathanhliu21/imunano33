/**
 * @file
 * @brief Contains imunano33::TempUnit and imunano33::PressureUnit enums
 */

#ifndef INCLUDE_IMUNANO33_UNIT_HPP_
#define INCLUDE_IMUNANO33_UNIT_HPP_

namespace imunano33 {
/**
 * @brief An enumerator describing units of temperature values
 */
enum TempUnit {
  FAHRENHEIT, //!< Converts temperature to Fahrenheit
  CELSIUS,    //!< Converts temperature to Celsius
  KELVIN      //!< Converts temperature to Kelvin
};

/**
 * @brief An enumerator describing units of pressure values
 */
enum PressureUnit {
  KPA,  //!< Converts pressure to kilopascal
  ATM,  //!< Converts pressure to standard atmospheric pressure, the pressure at
        //!< sea level and at 0°C
  MMHG, //!< Converts pressure to millimeters of mercury
  PSI   //!< Converts pressure to pounds per square inch
};
} // namespace imunano33

#endif
