/**
 * @file SmartWakeSleep.ino
 * @brief 智能唤醒/睡眠示例 - 组合显著运动检测和平面检测
 *
 * 本示例演示如何同时使用显著运动检测和平面检测来实现智能唤醒/睡眠功能：
 * - INT1: 显著运动检测 - 当设备被拿起或移动时唤醒
 * - INT2: 平面检测 - 当设备平放在桌面上时进入睡眠模式
 *
 * 应用场景：
 * - 智能手表/手环（拿起唤醒，放下睡眠）
 * - 移动设备电源管理
 * - 自动屏幕开关
 */

#include "DFRobot_BMI323.h"

#define BMI323_I2C_ADDR 0x69

DFRobot_BMI323 bmi323(&Wire, BMI323_I2C_ADDR);

// 中断标志
volatile bool gSigMotionFlag = false;
volatile bool gFlatFlag = false;

// 设备状态
enum DeviceState {
  STATE_SLEEP,
  STATE_AWAKE
};
DeviceState deviceState = STATE_SLEEP;

#if defined(ESP8266)
void IRAM_ATTR onSigMotionISR()
#else
void onSigMotionISR()
#endif
{
  gSigMotionFlag = true;
}

#if defined(ESP8266)
void IRAM_ATTR onFlatISR()
#else
void onFlatISR()
#endif
{
  gFlatFlag = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("BMI323 Smart Wake/Sleep Demo");
  Serial.println("============================");
  Serial.println("INT1: Significant Motion (Wake)");
  Serial.println("INT2: Flat Detection (Sleep)");
  Serial.println("\nPick up the board to wake, place flat to sleep.\n");

  while (!bmi323.begin()) {
    Serial.println("IMU init failed, retrying...");
    delay(1000);
  }

  // 配置加速度计：100Hz采样率，±2g量程
  bmi323.configAccel(bmi323.eAccelODR100Hz, bmi323.eAccelRange2G);

  // 配置显著运动检测（INT1）- 用于唤醒
  struct bmi3_sig_motion_config sigMotionCfg;
  sigMotionCfg.block_size = 200;
  sigMotionCfg.peak_2_peak_min = 30;
  sigMotionCfg.peak_2_peak_max = 30;
  sigMotionCfg.mcr_min = 0x10;  // 16
  sigMotionCfg.mcr_max = 0x10;  // 16

  if (!bmi323.enableSigMotionInterrupt(sigMotionCfg, bmi323.eINT1)) {
    Serial.println("Failed to enable sig-motion interrupt!");
    while (1) {
      delay(1000);
    }
  }

  // 配置平面检测（INT2）- 用于睡眠
  struct bmi3_flat_config flatCfg;
  flatCfg.theta = 9;              // Max tilt angle
  flatCfg.blocking = 3;           // Blocking mode
  flatCfg.hold_time = 50;         // 50 * 20ms = 1000ms
  flatCfg.hysteresis = 9;         // Hysteresis
  flatCfg.slope_thres = 0xCD;     // Slope threshold

  if (!bmi323.enableFlatInterrupt(flatCfg, bmi323.eINT2)) {
    Serial.println("Failed to enable flat interrupt!");
    while (1) {
      delay(1000);
    }
  }

  // 配置INT1中断引脚（显著运动）
#if defined(ESP32)
  pinMode(14 /*D6*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/), onSigMotionISR, FALLING);
#elif defined(ESP8266)
  pinMode(13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13), onSigMotionISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6), onSigMotionISR, FALLING);
#else
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, onSigMotionISR, FALLING);
#endif

  // 配置INT2中断引脚（平面检测）
#if defined(ESP32)
  pinMode(13 /*D7*/, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(13 /*D7*/), onFlatISR, FALLING);
#elif defined(ESP8266)
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), onFlatISR, FALLING);
#elif defined(ARDUINO_SAM_ZERO)
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), onFlatISR, FALLING);
#else
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, onFlatISR, FALLING);
#endif

  Serial.println("Connect BMI323 INT1 to pin 2 (or platform-specific pin)");
  Serial.println("Connect BMI323 INT2 to pin 3 (or platform-specific pin)");
  Serial.println("Device starts in SLEEP mode.\n");
}

void loop() {
  // 处理显著运动中断（唤醒）
  if (gSigMotionFlag) {
    gSigMotionFlag = false;
    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_SIG_MOTION) {
      if (deviceState == STATE_SLEEP) {
        deviceState = STATE_AWAKE;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println(">>> 设备唤醒 <<<");
        // 这里可以添加实际的唤醒操作，如点亮屏幕、启动传感器等
      }
    }
  }

  // 处理平面检测中断（睡眠）
  if (gFlatFlag) {
    gFlatFlag = false;
    uint16_t status = bmi323.getInterruptStatus();
    if (status & BMI3_INT_STATUS_FLAT) {
      if (deviceState == STATE_AWAKE) {
        deviceState = STATE_SLEEP;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println(">>> 设备进入睡眠模式 <<<");
        // 这里可以添加实际的睡眠操作，如关闭屏幕、降低功耗等
      }
    }
  }

  // 在唤醒状态下，可以执行其他任务
  if (deviceState == STATE_AWAKE) {
    // 例如：读取传感器数据、更新显示等
    // delay(100); // 避免过于频繁的操作
  }
}

