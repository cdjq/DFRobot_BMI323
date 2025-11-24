/**
 * @file FlatInterrupt.ino
 * @brief Demonstrates BMI323 flat (plane) detection interrupt.
 *
 * Connect INT1 (or INT2) of BMI323 to the MCU pin defined by intPin.
 * When the board is placed on a flat surface, the interrupt will trigger
 * and the sketch prints a log.
 *
 * Flat detection is useful for:
 * - Detecting when a device is placed on a table
 * - Orientation detection (flat vs. tilted)
 * - Power management (device is stationary)
 */

#include "DFRobot_BMI323.h"

#define BMI323_I2C_ADDR 0x69

DFRobot_BMI323 bmi323(&Wire, BMI323_I2C_ADDR);
volatile bool gFlatDetected = false;

#if defined(ESP8266)
void IRAM_ATTR onFlatISR()
#else
void onFlatISR()
#endif
{
  gFlatDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Flat Detection Interrupt Demo");
  Serial.println("Keep the board on a flat surface to trigger flat interrupt.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 使用官方示例的参数配置（参考 flat.c）
  // 结构体字段顺序：theta, blocking, hysteresis, hold_time, slope_thres
  struct bmi3_flat_config flatCfg;
  flatCfg.theta = 9;              // Max tilt angle: 64 * (tan(angle)^2), range 0-63
  flatCfg.blocking = 3;           // Blocking mode 3: Block if >1.5g or slope > threshold
  flatCfg.hold_time = 50;         // Min duration in flat position: 50 * 20ms = 1000ms
  flatCfg.hysteresis = 9;         // Hysteresis angle, range 0-255
  flatCfg.slope_thres = 0xCD;     // Min slope between samples: 0xCD = 205

  if (!bmi323.enableFlatInterrupt(flatCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable flat interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onFlatISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onFlatISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onFlatISR, FALLING);
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
  attachInterrupt(/*Interrupt No*/ 0, onFlatISR, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                             // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
}

void loop() {
  if (gFlatDetected) {
    gFlatDetected = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_FLAT) {
      Serial.print("Flat surface detected at ");
      Serial.print(millis());
      Serial.println(" ms");
    }
  }
}

