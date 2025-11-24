/**
 * @file StepCounter.ino
 * @brief Simple BMI323 pedometer example
 *
 * The BMI323 integrates a hardware step counter. This sketch shows how to
 * enable the feature and poll the accumulated step count once per second.
 */

#include "DFRobot_BMI323.h"

#define BMI323_I2C_ADDR 0x69

DFRobot_BMI323 bmi323(&Wire, BMI323_I2C_ADDR);
volatile bool gStepReady = false;

#if defined(ESP8266)
void IRAM_ATTR onStepInterrupt(void)
#else
void onStepInterrupt(void)
#endif
{
  gStepReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Step Counter Demo");
  Serial.println("========================");

  while (!bmi323.begin()) {
    Serial.println("I2C init failed, retry in 1s");
    delay(1000);
  }

  // 推荐配置：100Hz、±2g
  bmi323.configAccel(bmi323.eAccelODR100Hz,
                     bmi323.eAccelRange2G);

  if (!bmi323.enableStepCounterInterrupt(bmi323.eINT1)) {
    Serial.println("Enable step counter interrupt failed!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onStepInterrupt, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onStepInterrupt, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onStepInterrupt, FALLING);
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
  attachInterrupt(/*Interrupt No*/ 0, onStepInterrupt, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                                   // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif

  Serial.println("Connect BMI323 INT1 to the defined MCU pin.");
  Serial.println("Every step will trigger an interrupt and update the counter.\n");
}

void loop() {
  if (gStepReady) {
    gStepReady = false;

    uint16_t steps = 0;
    if (bmi323.readStepCounter(&steps) == BMI3_OK) {
      Serial.print("Steps: ");
      Serial.println(steps);
    }
  }
}
