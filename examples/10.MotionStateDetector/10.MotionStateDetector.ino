/**
 * @file MotionStateDetector.ino
 * @brief 运动状态检测示例 - 组合任意运动检测和静止检测
 *
 * 本示例演示如何同时使用任意运动检测和静止检测来判断设备的运动状态：
 * - INT1: 任意运动检测 - 当设备开始运动时触发
 * - INT2: 静止检测 - 当设备保持静止时触发
 *
 * 应用场景：
 * - 运动监测设备
 * - 活动状态追踪
 * - 节能管理（运动时激活，静止时休眠）
 */

#include "DFRobot_BMI323.h"

#define BMI323_I2C_ADDR 0x69

DFRobot_BMI323 bmi323(&Wire, BMI323_I2C_ADDR);

// 中断标志
volatile bool gAnyMotionFlag = false;
volatile bool gNoMotionFlag = false;

// 当前运动状态
enum MotionState {
  STATE_UNKNOWN,
  STATE_MOVING,
  STATE_STILL
};
MotionState currentState = STATE_UNKNOWN;

#if defined(ESP8266)
void IRAM_ATTR onAnyMotionISR()
#else
void onAnyMotionISR()
#endif
{
  gAnyMotionFlag = true;
}

#if defined(ESP8266)
void IRAM_ATTR onNoMotionISR()
#else
void onNoMotionISR()
#endif
{
  gNoMotionFlag = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Motion State Detector");
  Serial.println("============================");
  Serial.println("INT1: Any-Motion Detection");
  Serial.println("INT2: No-Motion Detection");
  Serial.println("Move the board to see state changes.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  // 配置加速度计：100Hz采样率，±2g量程
  // 100Hz采样率适合运动检测应用，提供足够的响应速度同时保持合理的功耗
  // ±2g量程适合日常运动检测场景，能够检测到较小的运动变化
  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 配置任意运动检测（INT1）
  struct bmi3_any_motion_config anyMotionCfg;
  anyMotionCfg.duration = 9;        // 9 * 20ms = 180ms
  anyMotionCfg.slope_thres = 9;     // 9 * 1.953mg ≈ 17.6mg
  anyMotionCfg.acc_ref_up = 1;      // Always update reference
  anyMotionCfg.hysteresis = 5;       // 5 * 1.953mg ≈ 9.8mg
  anyMotionCfg.wait_time = 4;       // 4 * 20ms = 80ms

  if (!bmi323.enableAnyMotionInterrupt(anyMotionCfg, bmi323.eINT1,
                                       bmi323.eAxisXYZ)) {
    Serial.println("Failed to enable any-motion interrupt!");
    while (1) {
      delay(1000);
    }
  }

  // 配置静止检测（INT2）
  struct bmi3_no_motion_config noMotionCfg;
  noMotionCfg.duration = 9;           // 9 * 20ms = 180ms
  noMotionCfg.slope_thres = 9;      // 9 * 1.953mg ≈ 17.6mg
  noMotionCfg.acc_ref_up = 1;        // Always update reference
  noMotionCfg.hysteresis = 5;         // 5 * 1.953mg ≈ 9.8mg
  noMotionCfg.wait_time = 5;          // 5 * 20ms = 100ms

  if (!bmi323.enableNoMotionInterrupt(noMotionCfg, bmi323.eINT2,
                                      bmi323.eAxisXYZ)) {
    Serial.println("Failed to enable no-motion interrupt!");
    while (1) {
      delay(1000);
    }
  }

  // 配置INT1中断引脚（任意运动）
#if defined(ESP32)
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/), onAnyMotionISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onAnyMotionISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6), onAnyMotionISR, FALLING);
#else
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, onAnyMotionISR, FALLING);
#endif

  // 配置INT2中断引脚（静止检测）
#if defined(ESP32)
  pinMode(13 /*D7*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13 /*D7*/), onNoMotionISR, FALLING);
#elif defined(ESP8266)
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), onNoMotionISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), onNoMotionISR, FALLING);
#else
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, onNoMotionISR, FALLING);
#endif

  Serial.println("Connect BMI323 INT1 to pin 2 (or platform-specific pin)");
  Serial.println("Connect BMI323 INT2 to pin 3 (or platform-specific pin)");
  Serial.println("Ready to detect motion state!\n");
}

void loop() {
  // 处理任意运动中断
  if (gAnyMotionFlag) {
    gAnyMotionFlag = false;
    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_ANY_MOTION) {
      if (currentState != STATE_MOVING) {
        currentState = STATE_MOVING;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println("我在运动");
      }
    }
  }

  // 处理静止中断
  if (gNoMotionFlag) {
    gNoMotionFlag = false;
    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_NO_MOTION) {
      if (currentState != STATE_STILL) {
        currentState = STATE_STILL;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println("我停下来了");
      }
    }
  }
}

