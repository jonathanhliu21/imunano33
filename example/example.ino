/**
 * This is an example of usage.
 *
 * This sketch reads data from the IMU and climate sensors and processes them
 * using IMUNano33. Depending on the USE_BLUETOOTH macro (see below), this data
 * is then communicated either via BLE or the serial port. Note that the BLE
 * communication is experimental and currently untested.
 *
 * Climate data is sent in the follwing form on the serial monitor:
 * "C<temperature in C>,<humidity in %>,<pressure in kPa>".
 * On BLE, it is sent on the 2A6E characteristic in the form:
 * "<temperature in C>,<humidity in %>,<pressure in kPa>".
 * It is important to note that these values are integers which are
 * 10 times their actual value, so there is one digit of precision.
 *
 * Quaternion/orientation data is sent in the following form on the serial monitor:
 * "Q<w>,<x>,<y>,<z>"
 * and like the following on BLE, on the 2A69 characteristic:
 * "<w>,<x>,<y>,<z>".
 * It is important to note that these values are integers which are
 * 1000 times their actual value, giving three digits of precision.
 *
 * For BLE, both characteristics are on the service 180A.
 */

#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

#include <ArduinoBLE.h>       // bluetooth
#include <Arduino_HTS221.h>   // temperature, huimdity
#include <Arduino_LPS22HB.h>  // pressure
#include <Arduino_LSM9DS1.h>  // accelerometer, gyroscope

#define IMUNANO33_EMBED
#include "imunano33.hpp"

#define COLOR_SHIFT_MS 10

// #define USE_BLUETOOTH        // uncomment this line to use bluetooth instead (experimental)
#define DEFAULT_SERIAL Serial1  // change this to Serial if using USB instead of TX & RX pins

// color LED control
unsigned char colors[3] = { 255, 0, 0 };
unsigned long long prevTimeColor;
enum ColorState {
  RED,
  YELLOW,
  GREEN,
  CYAN,
  BLUE,
  MAGENTA
} state{ RED };

// IMU control
float prevTimeIMU;

// climate control
float prevTimeClimate;

// IMU + climate control
imunano33::IMUNano33 proc;

// bluetooth communication control
BLEService service{ "180A" };
BLECharacteristic climateChr{ "2A6E", BLENotify, 32 };
BLECharacteristic imuChr{ "2A69", BLENotify, 32 };

void setup() {
  DEFAULT_SERIAL.begin(115200);

  while (!DEFAULT_SERIAL)
    ;
  while (!BARO.begin())
    ;
  while (!IMU.begin())
    ;
  while (!HTS.begin())
    ;

#ifdef USE_BLUETOOTH
  while (!BLE.begin())
    ;

  BLE.setLocalName("Arduino Nano 33 BLE Sense");  // local name
  BLE.setAdvertisedService(service);              // set advertised service

  // set characteristics of advertised service
  service.addCharacteristic(climateChr);
  service.addCharacteristic(imuChr);

  BLE.addService(service);  // add the service

  BLE.advertise();  // make it seen to others

  DEFAULT_SERIAL.println(BLE.address());  // print MAC address
#endif

  prevTimeIMU = millis() / 1000.0;
  proc.zeroIMU();
}

void loop() {
  // get current time
  float curTime = millis() / 1000.0;
  readIMU(curTime);

#ifndef USE_BLUETOOTH
  updateSerialIMU(proc.getRotQ());
#endif

  // climate data, update every ten seconds because it blocks
  if (curTime - prevTimeClimate >= 10.0) {
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();
    float pressure = BARO.readPressure();

    proc.updateClimate(temperature, humidity, pressure);

#ifndef USE_BLUETOOTH
    updateSerialClimate(temperature, humidity, pressure);
#endif

    prevTimeClimate = curTime;
  }

#ifdef USE_BLUETOOTH
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      float curTime = millis() / 1000.0;
      // read IMU data and process
      readIMU(curTime);

      // update characteristics
      updateBLEIMU(proc.getRotQ());

      // climate data, update every ten seconds because it blocks
      if (curTime - prevTimeClimate >= 10.0) {
        float temperature = HTS.readTemperature();
        float humidity = HTS.readHumidity();
        float pressure = BARO.readPressure();

        proc.updateClimate(temperature, humidity, pressure);
        updateBLEClimate(temperature, humidity, pressure);

        prevTimeClimate = curTime;
      }
    }
  }
#endif

  // shift color LED
  color();
}

void readIMU(float curTime) {
  // read IMU data
  float aX = 0;
  float aY = 0;
  float aZ = 0;
  float gX = 0;
  float gY = 0;
  float gZ = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(aX, aY, aZ);

    // need to invert to be consistent with axes on docs
    aY = -aY;
    aZ = -aZ;

    proc.updateIMUAccel({ aX, aY, aZ });
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gX, gY, gZ);

    // need to invert to be consistent with axes on docs
    gX = -gX;

    // converts degrees to radians
    gX *= (M_PI / 180);
    gY *= (M_PI / 180);
    gZ *= (M_PI / 180);

    proc.updateIMUGyro({ gX, gY, gZ }, curTime - prevTimeIMU);
    prevTimeIMU = curTime;
  }
}

void updateSerialIMU(const imunano33::Quaternion &qRot) {
  imunano33::Vector3D sendVec = qRot.vec() * 1000;

  DEFAULT_SERIAL.print("Q:");
  DEFAULT_SERIAL.print(lroundf(qRot.w() * 1000));
  DEFAULT_SERIAL.print(",");
  DEFAULT_SERIAL.print(lroundf(sendVec.x));
  DEFAULT_SERIAL.print(",");
  DEFAULT_SERIAL.print(lroundf(sendVec.y));
  DEFAULT_SERIAL.print(",");
  DEFAULT_SERIAL.print(lroundf(sendVec.z));
  DEFAULT_SERIAL.println();
}

// broadcasts climate data in celsius, %, and kilopascal
void updateSerialClimate(const float &temperature, const float &humidity, const float &pressure) {
  DEFAULT_SERIAL.print("C:");
  DEFAULT_SERIAL.print(lroundf(temperature * 10));
  DEFAULT_SERIAL.print(",");
  DEFAULT_SERIAL.print(lroundf(humidity * 10));
  DEFAULT_SERIAL.print(",");
  DEFAULT_SERIAL.print(lroundf(pressure * 10));
  DEFAULT_SERIAL.println();
}

void updateBLEIMU(const imunano33::Quaternion &qRot) {
  imunano33::Vector3D sendVec = qRot.vec() * 1000;
  String send = String{ lroundf(qRot.w() * 1000) } + "," + String{ lroundf(sendVec.x) } + "," + String{ lroundf(sendVec.y) } + "," + String{ lroundf(sendVec.z) };

  // +1 for null character at the end
  imuChr.writeValue(send.c_str(), send.length() + 1);
}

void updateBLEClimate(const float &temperature, const float &humidity, const float &pressure) {
  String send = String{ lroundf(temperature * 10) } + "," + String{ lroundf(humidity * 10) } + "," + String{ lroundf(pressure * 10) };

  // +1 for null character at the end
  climateChr.writeValue(send.c_str(), send.length() + 1);
}

void color() {
  unsigned long ms = millis();

  if (ms - prevTimeColor < 10) {
    return;
  }

  prevTimeColor = ms;

  switch (state) {
    case RED:
      {
        if (colors[0] == 255 && colors[1] == 0 && colors[2] == 0) {
          state = YELLOW;
          break;
        }
        colors[2]--;
        break;
      }
    case YELLOW:
      {
        if (colors[0] == 255 && colors[1] == 255 && colors[2] == 0) {
          state = GREEN;
          break;
        }
        colors[1]++;
        break;
      }
    case GREEN:
      {
        if (colors[0] == 0 && colors[1] == 255 && colors[2] == 0) {
          state = CYAN;
          break;
        }
        colors[0]--;
        break;
      }
    case CYAN:
      {
        if (colors[0] == 0 && colors[1] == 255 && colors[2] == 255) {
          state = BLUE;
          break;
        }
        colors[2]++;
        break;
      }
    case BLUE:
      {
        if (colors[0] == 0 && colors[1] == 0 && colors[2] == 255) {
          state = MAGENTA;
          break;
        }
        colors[1]--;
        break;
      }
    case MAGENTA:
      {
        if (colors[0] == 255 && colors[1] == 0 && colors[2] == 255) {
          state = RED;
          break;
        }
        colors[0]++;
        break;
      }
  }

  analogWrite(LEDR, colors[0]);
  analogWrite(LEDG, colors[1]);
  analogWrite(LEDB, colors[2]);
}
