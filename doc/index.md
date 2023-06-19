@mainpage
@tableofcontents

# Overview

IMUNano33 is a data processor library for the sensors on an Arduino Nano 33 BLE Sense, namely the:

* HTS221 for temperature and humidity
* LPS22HB for air pressure
* LSM9DS1 IMU, whose accelerometer and gyroscope are used

The HTS221 and LPS22HB are used for climate data, and the LSM9DS1 is used to determine the orientation of the Arduino Nano. It uses the gyroscope to integrate the angular rates and then corrects it with the direction of gravity given by the accelerometer. The magnetometer is not used to correct yaw because of high magnetic interference and noise in various applications, and because it is difficult to calibrate properly.

This library only processes the data and does not read in any data. It expects temperature (in °C), relative humidity, and air pressure (in kPa) from the climate sensors, and angular velocities about all three axes, accelerations in all three dimensions, and the time between the current and last measurement from the IMU. This data can be a combined input, or read from the climate sensors and the IMU separately (see imunano33::IMUNano33::update(), imunano33::IMUNano33::updateIMU(), and imunano33::IMUNano33::updateClimate()). Additionally, this library is not meant to be uploaded to the Arduino directly, because it uses the C++ standard library, which is not supported. Instead, it is meant to be used on a device such as a Raspberry Pi or a computer that the Arduino is connected to, where the input data from above is meant to be read from the BLE interface or from the serial port.

This library can also be used with an Arduino connected to an MPU-9250 or MPU-6050 IMU along with a DHT22 temperature/humidity sensor and a BMP390 pressure sensor. However, the axes mentioned in the documentation will not match. Additionally, as mentioned above, this library can be used as a standalone orientation calculator or a standalone climate data processor, so it can be used with just a MPU-9250/MPU-6050 or just a DHT22 + BMP390.


# Installation

There are three main ways to install this library

## Copying Folder

The most convenient way to install this library is to clone the repository, and copy the `include/imunano33` directory directly into your project directory, or in a folder designated for third party libraries. This way, you can include the main file, `imunano33/imunano33.hpp`, anywhere in your project.

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

The main class to use is the imunano33::IMUNano33 class. It provides independent methods for processing climate and IMU data, allowing you to ditch the climate or ditch the IMU if you do not have the data from one sensor or the other. However, it is important to know that this is only a data *processor*, and does not contain code for reading data input. This has to be done on your own. For example, you can use the Arduino's BLE interface or serial UART interface to read the data.

## Math Utilities

This library comes with some utility classes for math, namely svector::Vector3D, from my third-party vectors library, and imunano33::Quaternion, which is used for rotations.

The usage of the third-party vectors library can be found at the simplevectors [documentation page](https://jonyboi396825.github.io/simplevectors/a00123.html). The basic usage methods of the quaternion class can be found at the class link: imunano33::Quaternion. It should mainly be used for rotations, whose code can be found below:

```cpp
#include <imunano33/imunano33.hpp>

imunano33::Quaternion getRotationQuaternion() {
  // ...
}

int main() {
  imunano33::Quaternion quaternion = getRotationQuaternion();
  

  // Method 1, using the member function, if the quaternion is known
  svector::Vector3D vec{1, 0, 0};
  svector::Vector3D vecRotated1 = quaternion.rotate(vec);

  // Method 2, using the static member function, if the quaternion is unknown

  // 90 degrees (pi/2 radians) clockwise around x-axis
  vec = {0, 1, 0};
  svector::Vector3D axis{1, 0, 0};
  // make sure that the angle given is in radians
  svector::Vector3D vecRotated2 = Quaternion::rotate(vec, axis, M_PI / 2);
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

