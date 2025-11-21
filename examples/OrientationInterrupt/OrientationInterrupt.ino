/**
 * @file OrientationInterrupt.ino
 * @brief Demonstrates BMI323 orientation detection interrupt.
 *
 * Connect INT1 (or INT2) of BMI323 to the MCU pin defined by intPin.
 * When the board changes orientation (portrait/landscape, face up/down), the
 * interrupt will trigger and the sketch prints the new state.
 */

#include "DFRobot_BMI323.h"

DFRobot_BMI323 bmi323;
volatile bool gOrientationChanged = false;

#if defined(ESP8266)
void IRAM_ATTR onOrientationISR()
#else
void onOrientationISR()
#endif
{
  gOrientationChanged = true;
}

/* 
a. 0b00：portrait upright     竖屏直立 
b. 0b01：landscape left       向左横屏 
c. 0b10：portrait upside down 竖屏倒置 
d. 0b11：landscape right      向右横屏
*/
static void printOrientation(uint8_t portraitLandscape, uint8_t faceUpDown) {
  switch (portraitLandscape) {
    case BMI3_LANDSCAPE_LEFT:
      Serial.print("Landscape Left");
      break;
    case BMI3_LANDSCAPE_RIGHT:
      Serial.print("Landscape Right");
      break;
    case BMI3_PORTRAIT_UP_DOWN:
      Serial.print("Portrait Upside Down");
      break;
    case BMI3_PORTRAIT_UP_RIGHT:
      Serial.print("Portrait Upright");
      break;
    default:
      Serial.print("Unknown");
      break;
  }

  Serial.print(" / ");

  if (faceUpDown == BMI3_FACE_UP) {
    Serial.println("Face Up");
  } else if (faceUpDown == BMI3_FACE_DOWN) {
    Serial.println("Face Down");
  } else {
    Serial.println("Unknown Face Orientation");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Orientation Interrupt Demo");
  Serial.println("Rotate the board to see orientation updates.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 使用官方示例的参数配置（参考 orientation.c）
  struct bmi3_orientation_config orientCfg;
  orientCfg.ud_en = BMI3_ENABLE;      // Enable face up/down detection
  orientCfg.hold_time = 4;            // 4 * 20ms = 80ms
  orientCfg.hysteresis = 5;           // Accel hysteresis
  orientCfg.theta = 16;               // Max tilt angle
  orientCfg.mode = 1;                 // High-asymmetrical
  orientCfg.slope_thres = 30;         // Slope threshold
  orientCfg.blocking = 3;             // Block when movement is large

  if (!bmi323.enableOrientationInterrupt(orientCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable orientation interrupt!");
    while (1) {
      delay(1000);
    }
  }

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, onOrientationISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onOrientationISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, onOrientationISR, FALLING);
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
  attachInterrupt(/*Interrupt No*/ 0, onOrientationISR, FALLING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                                   // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif
}

void loop() {
  if (gOrientationChanged) {
    gOrientationChanged = false;

    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_ORIENTATION) {
      uint8_t pl = 0;
      uint8_t fud = 0;
      if (bmi323.readOrientation(&pl, &fud)) {
        Serial.print("Orientation: ");
        printOrientation(pl, fud);
      } else {
        Serial.println("Failed to read orientation data.");
      }
    }
  }
}

