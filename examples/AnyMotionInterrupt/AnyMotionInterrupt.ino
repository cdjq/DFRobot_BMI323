/**
 * @file AnyMotionInterrupt.ino
 * @brief Demonstrates BMI323 any-motion (slope) interrupt on INT1/INT2.
 *
 * Connect INT1 of BMI323 to the MCU pin defined by intPin. Whenever the
 * sensor detects motion exceeding the configured threshold for the specified
 * duration, an interrupt will fire and the sketch will log the event.
 */

#include "DFRobot_BMI323.h"

DFRobot_BMI323 bmi323;
volatile bool gMotionDetected = false;

#if defined(ESP8266)
void IRAM_ATTR onAnyMotionISR()
#else
void onAnyMotionISR()
#endif
{
  gMotionDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Any-Motion Interrupt Demo");
  Serial.println("Move the board to trigger an interrupt.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 使用官方示例的参数配置（参考 any_no_motion_enable_disable.c）
  // 结构体字段顺序：duration, slope_thres, acc_ref_up, hysteresis, wait_time
  struct bmi3_any_motion_config anyMotionCfg;
  anyMotionCfg.duration = 9;        // 9 * 20ms = 180ms
  anyMotionCfg.slope_thres = 9;      // 9 * 1.953mg ≈ 17.6mg
  anyMotionCfg.acc_ref_up = 1;       // Always update reference
  anyMotionCfg.hysteresis = 5;       // 5 * 1.953mg ≈ 9.8mg
  anyMotionCfg.wait_time = 4;        // 4 * 20ms = 80ms

  if (!bmi323.enableAnyMotionInterrupt(anyMotionCfg, bmi323.eINT1,
                                       bmi323.eAxisXYZ)) {
    Serial.println("Failed to enable any-motion interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onAnyMotionISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onAnyMotionISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onAnyMotionISR, FALLING);
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
  attachInterrupt(/*Interrupt No*/ 0, onAnyMotionISR, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                                 // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
}

void loop() {
  if (gMotionDetected) {
    gMotionDetected = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_ANY_MOTION) {
      Serial.print("Any-motion detected at ");
      Serial.print(millis());
      Serial.println(" ms");
    }
  }
}

