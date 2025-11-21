/**
 * @file IMUShow.ino
 * @brief Stream pitch/roll/yaw to the DFRobot_IMU_Show PC visualizer.
 *
 * Wire the BMI323 by I2C:
 *   VCC -> 3V3, GND -> GND, SDA/SCL -> MCU I2C pins, INT pins unused.
 *
 * The PC tool expects text lines in the format:
 *   "pitch:xx.xx roll:yy.yy yaw:zz.zz\n"
 * where the angles are in degrees.
 */

#include "DFRobot_BMI323.h"
#include <math.h>

DFRobot_BMI323 bmi323;

static const float kAccelScale = 1.0f / 16384.0f; // ±2g -> 16384 LSB per g
static const float kGyroScale = 250.0f / 32768.0f; // ±250 dps -> scale factor

float yawDeg = 0.0f;
uint32_t lastMicros = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("DFRobot IMU Show streaming demo");
  Serial.println("Output format: pitch:xx.xx roll:xx.xx yaw:xx.xx");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, check wiring. Retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);
  bmi323.configGyro(bmi323.eGyroODR100Hz, bmi323.eGyroRange250DPS);

  lastMicros = micros();
}

void loop() {
  int16_t raw[6] = {0};
  if (bmi323.getAccelGyroData(raw) == BMI3_OK) {
    uint32_t now = micros();
    float dt = (now - lastMicros) / 1e6f;
    lastMicros = now;

    float gx = raw[0] * kGyroScale;
    float gy = raw[1] * kGyroScale;
    float gz = raw[2] * kGyroScale;
    float ax = raw[3] * kAccelScale;
    float ay = raw[4] * kAccelScale;
    float az = raw[5] * kAccelScale;

    float pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    float roll = atan2f(-ay, az) * 57.29578f;

    yawDeg += gz * dt;
    if (yawDeg > 180.0f) yawDeg -= 360.0f;
    if (yawDeg < -180.0f) yawDeg += 360.0f;

    const float pitchOut = -yawDeg;
    const float rollOut = -roll;
    const float yawOut = -pitch;

    Serial.print("pitch:");
    Serial.print(pitchOut, 2);
    Serial.print(" roll:");
    Serial.print(rollOut, 2);
    Serial.print(" yaw:");
    Serial.print(yawOut, 2);
    Serial.print(" \n");
  }

  delay(10);
}

