/**
 * @file TiltInterrupt.ino
 * @brief Demonstrates BMI323 tilt detection interrupt.
 *
 * Connect INT1 (or INT2) of BMI323 to the MCU pin defined by intPin.
 * Tilt the board; when the tilt angle exceeds the configured threshold, an
 * interrupt fires and the sketch prints a log.
 */

#include "DFRobot_BMI323.h"

DFRobot_BMI323 bmi323;
volatile bool gTiltDetected = false;

#if defined(ESP8266)
void IRAM_ATTR onTiltISR()
#else
void onTiltISR()
#endif
{
  gTiltDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Tilt Interrupt Demo");
  Serial.println("Tilt the board to trigger the interrupt.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 使用官方示例的参数配置（参考 tilt.c）
  struct bmi3_tilt_config tiltCfg;
  tiltCfg.segment_size = 90;        // Averaging window (0-255)
  tiltCfg.min_tilt_angle = 200;     // Minimum tilt angle (0-255, value = 256 * cos(angle))
  tiltCfg.beta_acc_mean = 0x00FF;   // Low-pass smoothing coefficient

  if (!bmi323.enableTiltInterrupt(tiltCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable tilt interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onTiltISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onTiltISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onTiltISR, FALLING);
#else
  /* The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
   * ---------------------------------------------------------------------------------------
   * |                                        |  DigitalPin  | 2  | 3  |                   |
   * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  |                   |
   * |-------------------------------------------------------------------------------------|
   * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
   * |               Mega2560                 |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
   * |-------------------------------------------------------------------------------------|
   * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
   * |    Leonardo, other 32u4-based          |--------------------------------------------|
   * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
   * |--------------------------------------------------------------------------------------
   * ---------------------------------------------------------------------------------------------------------------------------------------------
   *                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
   * ---------------------------------------------------------------------------------------------------------------------------------------------
   * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
   * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
   * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
   * |-------------------------------------------------------------------------------------------------------------------------------------------|
   */
  pinMode(2, INPUT_PULLUP);  // UNO/Mega2560 use pin 2, Leonardo uses pin 3
  attachInterrupt(/*Interrupt No*/ 0, onTiltISR, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                             // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
}

void loop() {
  if (gTiltDetected) {
    gTiltDetected = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_TILT) {
      Serial.print("Tilt detected at ");
      Serial.print(millis());
      Serial.println(" ms");
    }
  }
}

