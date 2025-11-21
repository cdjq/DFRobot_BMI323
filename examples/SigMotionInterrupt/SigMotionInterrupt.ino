/**
 * @file SigMotionInterrupt.ino
 * @brief Demonstrates BMI323 significant motion (sig-motion) interrupt.
 *
 * Connect INT1 (or INT2) of BMI323 to the MCU pin defined by intPin.
 * When the board moves in the same direction (significant motion), the
 * interrupt will trigger and the sketch prints a log.
 *
 * Significant motion is different from any-motion:
 * - Any-motion: Detects any movement exceeding threshold
 * - Sig-motion: Detects sustained movement in the same direction
 */

#include "DFRobot_BMI323.h"

DFRobot_BMI323 bmi323;
volatile bool gSigMotionDetected = false;

#if defined(ESP8266)
void IRAM_ATTR onSigMotionISR()
#else
void onSigMotionISR()
#endif
{
  gSigMotionDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Significant Motion Interrupt Demo");
  Serial.println("Move the board in the same direction to trigger sig-motion.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 使用官方示例的参数配置（参考 sig_motion.c）
  // 结构体字段顺序：block_size, peak_2_peak_min, mcr_min, peak_2_peak_max, mcr_max
  struct bmi3_sig_motion_config sigMotionCfg;
  sigMotionCfg.block_size = 200;        // Size of segment for detection (0-65535)
  sigMotionCfg.peak_2_peak_min = 30;    // Min peak-to-peak acceleration (0-1023)
  sigMotionCfg.peak_2_peak_max = 30;    // Max peak-to-peak acceleration (0-1023)
  sigMotionCfg.mcr_min = 0x10;          // Min mean crossing rate per second (0-62)
  sigMotionCfg.mcr_max = 0x10;          // Max mean crossing rate per second (0-62)

  if (!bmi323.enableSigMotionInterrupt(sigMotionCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable sig-motion interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onSigMotionISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onSigMotionISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onSigMotionISR, FALLING);
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
  attachInterrupt(/*Interrupt No*/ 0, onSigMotionISR, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                                 // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
}

void loop() {
  if (gSigMotionDetected) {
    gSigMotionDetected = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_SIG_MOTION) {
      Serial.print("Significant motion detected at ");
      Serial.print(millis());
      Serial.println(" ms");
    }
  }
}

