@mainpage
@tableofcontents

# Overview

IMUNano33 is a data processor library for the sensors on an Arduino Nano 33 BLE Sense, namely the:

* HTS221 for temperature and humidity
* LPS22HB for air pressure
* LSM9DS1 IMU, whose accelerometer and gyroscope are used

The HTS221 and LPS22HB are used for climate data, and the LSM9DS1 is used to determine the orientation of the Arduino Nano. It uses the gyroscope to integrate the angular rates and then corrects it with the direction of gravity given by the accelerometer. The magnetometer is not used to correct yaw because of high magnetic interference and noise in various applications, and because it is difficult to calibrate properly.

This library only processes the data and does not read in any data. It expects temperature (in °C), relative humidity, and air pressure (in kPa) from the climate sensors, and angular velocities about all three axes, accelerations in all three dimensions, and the time between the current and last measurement from the IMU. This data can be a combined input, or read from the climate sensors and the IMU separately (see imunano33::IMUNano33::update(), imunano33::IMUNano33::updateIMU(), and imunano33::IMUNano33::updateClimate()). This library can be used on a device such as a Raspberry Pi, which supports the C++ standard library, or it can be used on the Arduino with the macro `IMUNANO33_EMBED` defined **before** the include statement.

This library can also be used with an Arduino connected to an MPU-9250 or MPU-6050 IMU along with a DHT22 temperature/humidity sensor and a BMP390 pressure sensor. However, the axes mentioned in the documentation will not match. Additionally, as mentioned above, this library can be used as a standalone orientation calculator or a standalone climate data processor, so it can be used with just a MPU-9250/MPU-6050 or just a DHT22 + BMP390.

## Links

* Source Code: https://github.com/jonathanhliu21/imunano33
* Documentation: https://jonathanhliu21.github.io/imunano33/index.html

# Installation

There are four main ways to install this library.

## Single Include

The easiest way to install this library is to download the single include file on the GitHub releases page and place it into a convenient place in your project. The examples below show the include path to be `imunano33/imunano33.hpp`, but replace this with the include path that you are using to add the library.

## Copying Folder

Another way to install this library is to clone the repository, and copy the `include/imunano33` directory directly into your project directory, or in a folder designated for third party libraries. This way, you can include the main file, `imunano33/imunano33.hpp`, anywhere in your project.

## CMake Submodule

Alternatively, you can use CMake to install this library. The include path would be `imunano33/imunano33.hpp` for this option. Clone the repository or add it as a submodule, and then type in the following in your CMakeLists.txt:

```cmake
# other build code

add_subdirectory(/path/to/imunano33)
target_link_libraries(your_target PUBLIC imunano33::imunano33)

# other build code
```

## CMake Global Install

Another alternative is installing the library to the system. First, type in this script:

```sh
$ cmake -B build -DCMAKE_BUILD_TYPE=Release
$ cmake --build build --target install
```

Then, add this to your CMakeLists.txt:

```cmake
# other build code

find_package(imunano33 REQUIRED)
target_link_libraries(your_target PUBLIC imunano33::imunano33)

# other build code
```

The include path will be `imunano33/imunano33.hpp`.

## Embedding on an Arduino

As of version 0.1.x, this library can be embedded onto an Arduino.

The only way to install this library with an Arduino is downloading the single include header file from the releases on the GitHub page and copying it into your Arduino project directory.

After installing, it is important to add the following macro to the top of your code, before your include statement. This macro allows the library to know that it is used on an Arduino, without the C++ standard library. Without adding this, the program will not compile.

```cpp
#define IMUNANO33_EMBED
#include "imunano33.hpp"

imunano33::IMUNano33 processor;

void setup() {
  // ...
}

void loop() {
  // ...
}
```

It is important to note that all class and method names are the same regardless of whether the macro is set. The differences are that the embedded library does not use the `std` namespace or classes/functions from the C++ standard library, it uses `float` rather than `double` as its primary number type, and it uses svector::EmbVec3D instead of svector::Vector3D as its primary vector type.

# Theory

This section explains the math behind how this library works. Most of the math for the quaternions and the complementary filter are from these resources:

* https://jerabaul29.github.io/assets/quaternions/quaternions.pdf
* https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf
* https://stanford.edu/class/ee267/lectures/lecture10.pdf

In case these links go down, below contains a brief explanation of the math in the links above.

## Axes Definitions

Below shows a diagram and a description of the axes definitions.

With the Arduino flat on a table and the sensors facing up and the opening of the Micro USB port facing towards the front, the positive x direction points towards the front, the positive y direction points perpendicular and to the left, and the positive z direction points directly up. Note that the Arduino may not measure the angular velocities and accelerations with respect to these axes, so you may need to correct the measured values. Positive rotations about an axis mean counterclockwise rotations when viewed from the direction that the axis is pointing.

@image html axes.jpeg width=50%

@note the LSM9DS1 measurements may not be consistent with these axes, so you may need to convert the measurements to fit the axes.

## Quaternions

A quaternion consists of 4 components, one of which is the scalar component and the other three are which are the vector component.

@f[
q = [q_0, q_1, q_2, q_3] \\
q = [q_0, \vec{v}] \\
\vec{v} = [q_1, q_2, q_3]
@f]

### Definitions

The **norm** of a quaternion is defined as @f$l = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^3}@f$. A unit quaternion has a norm of 1. 

The **conjugate** of a quaternion is defined as @f$q^{\star} = [q_0,-q_1,-q_2,-q_3]@f$.

The **inverse** of a quaternion is defined as @f$q^{-1}=\frac{q^{\star}}{l^2}@f$. For a unit quaternion, the inverse equals the conjugate.

The **product** of two quaternions (important for rotations):

@f[
[w_1, \vec{v_1}]\cdot[w_2, \vec{v_2}]=[w_1w_2-\vec{v_1} \cdot \vec{v_2}, w_1 \vec{v_2}+w_2 \vec{v_1}+ \vec{v_1}\times \vec{v_2}]
@f]

### Rotating Vectors

A 3D rotation can be described as a unit quaternion. If @f$\vec{v_A}@f$ is the axis of rotation (must be a nonzero vector), and the angle of rotation around this axis is @f$\theta@f$, then the quaternion that can describe this rotation is: @f$q_R = [\cos(\frac{\theta}{2}), \sin(\frac{\theta}{2})\frac{\vec{v_A}}{||\vec{v_A}||}]@f$

Applying a rotation quaternion to a vector @f$\vec{v}@f$: @f$[0,\vec{v_R}]=q_R\cdot[0,\vec{v}]\cdot q_R^{\star}@f$, where @f$\vec{v_R}@f$ is the rotated vector.

## Complementary Filter

This section goes over the complementary filter that combines the gyroscope and accelerometer data from the IMU. The magnetometer is not used because it is difficult to calibrate, and it needs to be calibrated differently based on different locations due to unpredictable noise.

The sub-sections below go over the steps of the complementary filter.

### Gyro Integration

Given the angular velocity vector (in all three rotation axes), @f$\vec{\omega}@f$, the delta quaternion is defined as @f$q_\Delta = [\Delta{t}||\vec{\omega}||, \frac{\vec{\omega}}{||\vec{\omega}||}]@f$, where @f$\Delta{t}@f$ is the time between measurements. If @f$\vec{\omega}@f$ is near zero, then this step is skipped. To get the integrated quaternion: @f$q_{t\omega}=q_{t-1}q_{\Delta}@f$, where @f$t@f$ is the current time step and @f$t-1@f$ is the previous time step.

### Angle Correction

Angle correction is done by the accelerometer by taking a fraction of its measurement, assuming that its measurement points towards the direction of the ground. Because of frequent movement, this fraction is often a very small value. The fraction is given as `1 - gyroFavoring` (or `1 - favoring`) in the code. The acceleration vector @f$\vec{a}@f$ is measured by the accelerometer, which is then rotated into world frame from body frame using @f$q_{t\omega}@f$, becoming @f$\vec{a_R}@f$. This vector is then rotated in the direction of the world frame gravity vector @f$\vec{g}=[0,0,-1]@f$. To do this, the axis of rotation @f$\vec{v_R}@f$ is determined by crossing @f$\vec{a_R}\times\vec{g}@f$, and the angle @f$\theta@f$ is determined by @f$\arccos{\left(\frac{\vec{a_R}\cdot\vec{g}}{||\vec{a_R}||||\vec{g}||}\right)}@f$. If either the angle or the axis of rotation vector are zero, then this step is skipped. To have this rotation applied to the original quaternion, we create a rotation quaternion @f$q_a@f$ with an angle of @f$\theta@f$ multiplied by one minus the gyro favoring (`1 - gyroFavoring` in the code), and the axis of rotation being @f$\frac{\vec{v_R}}{||\vec{v_R}||}@f$. This quaternion would then be multiplied by @f$q_{t\omega}@f$, given above, to give us the final rotation quaternion for the current time step @f$t@f$: @f$q_{t\omega}q_a=q_t@f$.

# Usage

The main class to use is the imunano33::IMUNano33 class. It provides independent methods for processing climate and IMU data, allowing you to ditch the climate or ditch the IMU if you do not have the data from one sensor or the other. However, it is important to know that this is only a data *processor*, and does not contain code for reading data input. This has to be done on your own.

## Math Utilities

This library comes with some utility classes for math, namely svector::Vector3D, from my third-party vectors library, and imunano33::Quaternion, which is used for rotations.

The usage of the third-party vectors library can be found at the simplevectors [documentation page](https://jonathanhliu21.github.io/simplevectors/a00123.html). The basic usage methods of the quaternion class can be found at the class link: imunano33::Quaternion. It should mainly be used for rotations, whose code can be found below:

```cpp
#include <cmath>

#include <imunano33/imunano33.hpp>

imunano33::Quaternion getRotationQuaternion() {
  // ...
}

int main() {
  imunano33::Quaternion quaternion = getRotationQuaternion();
  

  // Method 1: using the member function, if the quaternion is known
  svector::Vector3D vec{1, 0, 0};
  svector::Vector3D vecRotated1 = quaternion.rotate(vec);

  // Method 2: using the static member function, if the quaternion is unknown

  // 90 degrees (pi/2 radians) clockwise around x-axis
  vec = {0, 1, 0};
  svector::Vector3D axis{1, 0, 0};
  // make sure that the angle given is in radians
  svector::Vector3D vecRotated2 = imunano33::Quaternion::rotate(vec, axis, M_PI / 2);

  // Method 3: constructing a quaternion, then using the member function
  imunano33::Quaternion quaternion2{axis, M_PI / 2};
  svector::Vector3D vecRotated3 = quaternion2.rotate(vec); 
}
```

Note that unless otherwise specified, all methods that ask you to input an angle are in radians, so you will have to convert degrees to radians.

## Initialization

There are three different ways to initialize the class. The default constructor makes the gyroscope favoring (mentioned above) to be 0.98, and the rotation angle of the initial rotation quaternion to be zero (initial position determines world frame axes; in other words, future measurements will be relative to the initial position). The first constructor allows you to specify the gyroscope favoring, while the rotation angle of the initial rotation quaternion is still zero. The second constructor allows you to specify the gyroscope favoring and the initial rotation quaternion, where it assumes that the Arduino was rotated before measuring begins.

```cpp
#include <cmath>

#include <imunano33/imunano33.hpp>

int main() {
  // default constructor
  imunano33::IMUNano33 proc1;

  // constructor 1 (gyro favoring)
  imunano33::IMUNano33 proc2{0.99};

  // constructor 2 (gyro favoring, initial quaternion)
  // this example shows an initial counterclockwise rotation around the x-axis
  imunano33::IMUNano33 proc3{0.98, imunano33::Quaternion{svector::Vector3D{1, 0, 0}, M_PI / 2}};
}
```

## Climate

To update the climate, use imunano33::IMUNano33::updateClimate(). It expects temperature, humidity, and pressure readings from the sensor. The table below shows the units of each quantity that you should measure before passing in the quantity into imunano33::IMUNano33::updateClimate().

Quantity | Unit
-------- | ------
Temperature | Celsius
Relative Humidity | Percent
Barometric Pressure | Kilopascal

Below is an example of updating the data processor with climate data:

```cpp
#include <imunano33/imunano33.hpp>

double readTempC() {
  // ...
}

double readRelativeHumidity() {
  // ...
}

double readPressurekPa() {
  // ...
}

int main() {
  imunano33::IMUNano33 proc;
  
  while (true) {
    double curTemp = readTempC();
    double curHumidity = readRelativeHumidity();
    double curPressure = readPressurekPa();

    proc.updateClimate(curTemp, curHumidity, curPressure);
  }
}
```

The data processor can convert the current climate data into the following units, shown in the table below:

Quantity | Unit
-------- | ------
Temperature | Celsius, Fahrenheit, Kelvin
Relative Humidity | Percent
Barometric Pressure | Kilopascal, Standard Atmospheres, Millimeters of Mercury, Pounds per Square Inch

Below is an example of converting units.

```cpp
#include <imunano33/imunano33.hpp>

double readTempC() {
  // ...
}

double readRelativeHumidity() {
  // ...
}

double readPressurekPa() {
  // ...
}

int main() {
  imunano33::IMUNano33 proc;
  
  while (true) {
    double curTemp = readTempC();
    double curHumidity = readRelativeHumidity();
    double curPressure = readPressurekPa();

    proc.updateClimate(curTemp, curHumidity, curPressure);

    double tempC = proc.getTemperature<imunano33::CELSIUS>();
    double tempF = proc.getTemperature<imunano33::FAHRENHEIT>();
    double tempK = proc.getTemperature<imunano33::KELVIN>();

    double relativeHumidity = proc.getHumidity();

    double pressureKPa = proc.getPressure<imunano33::KPA>();
    double pressureAtm = proc.getPressure<imunano33::ATM>();
    double pressureMmHg = proc.getPressure<imunano33::MMHG>();
    double pressurePSI = proc.getPressure<imunano33::PSI>();
  }
}
```

## IMU

To update the orientation data, use imunano33::IMUNano33::updateIMU(). To use this method, make sure that the readings are consistent with the axes defined above. The unit of the accelermoter readings can be anything, but meters per second squared is recommended because it is in SI units. The unit of the gyroscope readings **must** be in radians per second. Usually, they are in degrees per second, so they must be converted. The time difference between measurement readings **must** be in seconds. Below is an example of updating IMU data:

```cpp
#include <imunano33/imunano33.hpp>

svector::Vector3D readGyro() {
  // ...
  // returns <roll, pitch, yaw>
}

svector::Vector3D readAcc() {
  // ...
}

double getCurTime() {
  // ...
  // returns time, in seconds
}

int main() {
  imunano33::IMUNano33 proc;

  double prevTime = getCurTime();
  
  while (true) {
    svector::Vector3D gyro = readGyro();
    svector::Vector3D acc = readAcc();
    svector::Vector3D curTime = getCurTime();

    proc.updateIMU(acc, gyro, curTime - prevTime);
    
    prevTime = curTime;
  }
}
```

If the accelerometer values and gyroscope values are sampled separately, then the imunano33::IMUNano33::updateIMUAccel() and imunano33::IMUNano33::updateIMUGyro() methods can be used to update both values on their own:

```cpp
#include <imunano33/imunano33.hpp>

svector::Vector3D readGyro() {
  // ...
  // returns <roll, pitch, yaw>
}

svector::Vector3D readAcc() {
  // ...
}

double getCurTime() {
  // ...
  // returns time, in seconds
}

int main() {
  imunano33::IMUNano33 proc;

  double prevTime = getCurTime();
  
  while (true) {
    svector::Vector3D gyro = readGyro();
    svector::Vector3D acc = readAcc();
    svector::Vector3D curTime = getCurTime();

    // This is the same as calling proc.updateIMU() in this case
    // because they are sampled at the same time. If the accelerometer
    // and gyro are sampled at different times, each method should
    // be called separately at those times.
    proc.updateIMUGyro(gyro, curTime - prevTime);
    proc.updateIMUAccel(acc);
    
    prevTime = curTime;
  }
}
```

@note imunano33::IMUNano33::updateIMUAccel() and imunano33::IMUNano33::updateIMUGyro() are *not* commutative. imunano33::IMUNano33::updateIMUGyro() *sets* the latest known orientation, while imunano33::IMUNano33::updateIMUAccel() *corrects* the latest known orientation. If imunano33::IMUNano33::updateIMUGyro() is called first, then it rotates the Arduino first and sets the new orientation. Then, this new orientation is corrected with the imunano33::IMUNano33::updateIMUAccel() call. If imunano33::IMUNano33::updateIMUAccel() is called first, then it corrects the *previous* orientation, and then the Arduino is rotated to a *new*, *uncorrected* orientation with imunano33::IMUNano33::updateIMUGyro(). imunano33::IMUNano33::updateIMU() calls imunano33::IMUNano33::updateIMUGyro() first, and then it corrects the orientation with imunano33::IMUNano33::updateIMUAccel().

To get the current rotation quaternion, use imunano33::IMUNano33::getRotQ().

```cpp
#include <imunano33/imunano33.hpp>

svector::Vector3D readGyro() {
  // ...
  // returns <roll, pitch, yaw>
}

svector::Vector3D readAcc() {
  // ...
}

double getCurTime() {
  // ...
  // returns time, in seconds
}

int main() {
  imunano33::IMUNano33 proc;

  double prevTime = getCurTime();
  
  while (true) {
    svector::Vector3D gyro = readGyro();
    svector::Vector3D acc = readAcc();
    svector::Vector3D curTime = getCurTime();

    proc.updateIMU(acc, gyro, curTime - prevTime);
    
    prevTime = curTime;

    imunano33::Quaternion curQ = proc.getRotQ();

    // Gets X, Y, Z axes of Arduino relative to world frame.
    // Does this by rotating the body fram XYZ axes using the rotation 
    // quaternion given above.
    svector::Vector3D iBodyFrame{1, 0, 0};
    svector::Vector3D jBodyFrame{0, 1, 0};
    svector::Vector3D kBodyFrame{0, 0, 1};

    svector::Vector3D iWorldFrame = curQ.rotate(iBodyFrame);
    svector::Vector3D jWorldFrame = curQ.rotate(jBodyFrame);
    svector::Vector3D kWorldFrame = curQ.rotate(kBodyFrame);
  }
}
```

## IMU and Climate

If both IMU and climate data are known, then use imunano33::IMUNano33::update(), which takes in both climate and IMU data inputs. For specifications of the inputs, read the sections above. Below shows an example of using the method:


```cpp
#include <imunano33/imunano33.hpp>

svector::Vector3D readGyro() {
  // ...
  // returns <roll, pitch, yaw>
}

svector::Vector3D readAcc() {
  // ...
}

double getCurTime() {
  // ...
  // returns time, in seconds
}

double readTempC() {
  // ...
}

double readRelativeHumidity() {
  // ...
}

double readPressurekPa() {
  // ...
}

int main() {
  imunano33::IMUNano33 proc;

  double prevTime = getCurTime();
  
  while (true) {
    svector::Vector3D gyro = readGyro();
    svector::Vector3D acc = readAcc();
    svector::Vector3D curTime = getCurTime();

    double curTemp = readTempC();
    double curHumidity = readRelativeHumidity();
    double curPressure = readPressurekPa();

    proc.update(acc, gyro, curTime - prevTime, curTemp, curHumidity, curPressure);
    // above line is equivalent to below two lines
    // proc.updateIMU(acc, gyro, curTime - prevTime);  
    // proc.updateClimate(curTemp, curHumidity, curPressure);
    prevTime = curTime;

    imunano33::Quaternion curQ = proc.getRotQ();

    // Gets X, Y, Z axes of Arduino relative to world frame.
    // Does this by rotating the body fram XYZ axes using the rotation 
    // quaternion given above.
    svector::Vector3D iBodyFrame{1, 0, 0};
    svector::Vector3D jBodyFrame{0, 1, 0};
    svector::Vector3D kBodyFrame{0, 0, 1};

    svector::Vector3D iWorldFrame = curQ.rotate(iBodyFrame);
    svector::Vector3D jWorldFrame = curQ.rotate(jBodyFrame);
    svector::Vector3D kWorldFrame = curQ.rotate(kBodyFrame);

    // climate processing
    double tempC = proc.getTemperature<imunano33::CELSIUS>();
    double tempF = proc.getTemperature<imunano33::FAHRENHEIT>();
    double tempK = proc.getTemperature<imunano33::KELVIN>();

    double relativeHumidity = proc.getHumidity();

    double pressureKPa = proc.getPressure<imunano33::KPA>();
    double pressureAtm = proc.getPressure<imunano33::ATM>();
    double pressureMmHg = proc.getPressure<imunano33::MMHG>();
    double pressurePSI = proc.getPressure<imunano33::PSI>();
  }
}
```

## More usage information

More usage information can be found at the imunano33::IMUNano33 class documentation page. Note that these documentation pages are generated without the `IMUNANO33_EMBED` macro and are meant to be used with the Raspberry Pi. However, it is important to note that all class and method names are the same regardless of whether the macro is set. The differences are that it does not use the `std` namespace or classes/functions from the C++ standard library, it uses `float` rather than `double` as its primary number type, and it uses svector::EmbVec3D instead of svector::Vector3D as its primary vector type.

# License

The MIT License (MIT)

Copyright © 2023 Jonathan Liu

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
