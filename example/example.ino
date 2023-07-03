#include <ArduinoBLE.h>      // bluetooth
#include <Arduino_HTS221.h>  // temperature, huimdity
#include <Arduino_LPS22HB.h> // pressure
#include <Arduino_LSM9DS1.h> // accelerometer, gyroscope

// #define IMUNANO33_EMBED
// #include "imunano33.hpp"

#define COLOR_SHIFT_MS 10

// color control
unsigned char colors[3] = {255, 0, 0};
unsigned long long prevTimeColor;
enum ColorState {
  RED,
  YELLOW,
  GREEN,
  CYAN,
  BLUE,
  MAGENTA
} state{ RED };

// imu control
float prevTimeIMU;

void setup() {
  Serial.begin(115200);

  while (!BARO.begin())
    ;
  while (!IMU.begin())
    ;
  while (!HTS.begin())
    ;

  prevTimeIMU = millis() / 1000.0;
}

void loop() {
  // get current time
  float curTime = millis() / 1000.0;

  // read IMU data
  float aX = 0;
  float aY = 0; //<-- need to invert
  float aZ = 0; //<-- need to invert
  float gX = 0; //<-- need to invert
  float gY = 0;
  float gZ = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(aX, aY, aZ);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gX, gY, gZ);
    Serial.print(gX, 5);
    Serial.print(",");
    Serial.print(gY, 5);
    Serial.print(",");
    Serial.print(gZ, 5);
    Serial.println();
  }

 

  prevTimeIMU = curTime;

  // shift color LED
  color();
}

void color() {
  unsigned long long ms = millis();

  if (ms - prevTimeColor < 10) {
    return;
  }

  prevTimeColor = ms;

  switch (state) {
  case RED: {
    if (colors[0] == 255 && colors[1] == 0 && colors[2] == 0) {
      state = YELLOW;
      break;
    }
    colors[2]--;
    break;
  }
  case YELLOW: {
    if (colors[0] == 255 && colors[1] == 255 && colors[2] == 0) {
      state = GREEN;
      break;
    }
    colors[1]++;
    break;
  }
  case GREEN: {
    if (colors[0] == 0 && colors[1] == 255 && colors[2] == 0) {
      state = CYAN;
      break;
    }
    colors[0]--;
    break;
  }
  case CYAN: {
    if (colors[0] == 0 && colors[1] == 255 && colors[2] == 255) {
      state = BLUE;
      break;
    }
    colors[2]++;
    break;
  }
  case BLUE: {
    if (colors[0] == 0 && colors[1] == 0 && colors[2] == 255) {
      state = MAGENTA;
      break;
    }
    colors[1]--;
    break;
  }
  case MAGENTA: {
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
