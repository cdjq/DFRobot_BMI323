/**
 * @file TapInterrupt.ino
 * @brief Demonstrates BMI323 tap detection (single/double/triple) interrupt.
 *
 * Connect INT2 (default) of BMI323 to the MCU pin defined by intPin.
 * Tap the sensor gently to trigger single/double/triple tap events.
 */

#include "DFRobot_BMI323.h"

DFRobot_BMI323 bmi323;
volatile bool gTapPending = false;

#if defined(ESP8266)
void IRAM_ATTR onTapISR()
#else
void onTapISR()
#endif
{
  gTapPending = true;
}

static void printTap(uint8_t mask) {
  if (mask & BMI3_TAP_DET_STATUS_SINGLE) {
    Serial.println("Single tap detected.");
  }
  if (mask & BMI3_TAP_DET_STATUS_DOUBLE) {
    Serial.println("Double tap detected.");
  }
  if (mask & BMI3_TAP_DET_STATUS_TRIPLE) {
    Serial.println("Triple tap detected.");
  }
  if (mask == 0) {
    Serial.println("Tap interrupt, but no tap flag set.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Tap Interrupt Demo");
  Serial.println("Tap the sensor to trigger single/double/triple tap.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR400Hz, bmi323.eAccelRange4G);

  // 使用官方示例的参数配置（参考 tap.c）
  struct bmi3_tap_detector_config tapCfg;
  memset(&tapCfg, 0, sizeof(tapCfg));
  tapCfg.axis_sel = 1;                    // 使用Y轴
  tapCfg.max_dur_between_peaks = 5;
  tapCfg.max_gest_dur = 0x11;
  tapCfg.max_peaks_for_tap = 5;
  tapCfg.min_quite_dur_between_taps = 7;
  tapCfg.mode = 1;                        // Normal
  tapCfg.quite_time_after_gest = 5;
  tapCfg.tap_peak_thres = 0x2C;
  tapCfg.tap_shock_settling_dur = 5;
  tapCfg.wait_for_timeout = 1;            // Gesture confirmation

  if (!bmi323.enableTapInterrupt(tapCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable tap interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onTapISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onTapISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onTapISR, FALLING);
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
  pinMode(3, INPUT_PULLUP);  // UNO/Mega2560 use pin 3, Leonardo uses pin 2
  attachInterrupt(/*Interrupt No*/ 1, onTapISR, FALLING);  // Open the external interrupt 1, connect INT1/2 to the digital pin of the main control:
                                                           // UNO(3), Mega2560(3), Leonardo(2), microbit(P1).
#endif
}

void loop() {
  if (gTapPending) {
    gTapPending = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_TAP) {
      uint8_t tapMask = 0;
      if (bmi323.readTapStatus(&tapMask)) {
        printTap(tapMask);
      } else {
        Serial.println("Failed to read tap status.");
      }
    }
  }
}

