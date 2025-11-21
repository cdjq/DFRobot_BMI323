/**
 * @file  DFRobot_BMI323.h
 * @brief 定义DFRobot_BMI323类的基础结构
 * @n     这是一个6轴IMU传感器（加速度计+陀螺仪），可以通过I2C接口进行控制。
 * @n BMI323具有多种运动检测功能，如计步器、任意运动检测、静止检测、敲击检测等。
 * @n     这些功能使得BMI323传感器在可穿戴设备、智能设备等应用中非常有用。
 * @n     同样，BMI323具有中断引脚，可以在不使用软件算法的情况下以节能方式使用。
 * @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      DFRobot
 * @version     V1.0.0
 * @date        2024-01-01
 * @url         https://github.com/DFRobot/DFRobot_BMI323
 */

#ifndef __DFRobot_BMI323_H
#define __DFRobot_BMI323_H

#include "Arduino.h"
#include "Wire.h"
#include "bmi323.h"
#include "stdint.h"

/** 芯片ID定义 */
#define BMI323_CHIP_ID 0x43 ///< BMI323芯片ID

class DFRobot_BMI323 {
public:
  /**
   * @enum eAccelRange_t
   * @brief 加速度计量程枚举
   * @details 定义加速度计的测量量程，单位：g（重力加速度）
   */
  typedef enum {
    eAccelRange2G = BMI3_ACC_RANGE_2G,  ///< ±2g量程
    eAccelRange4G = BMI3_ACC_RANGE_4G,  ///< ±4g量程
    eAccelRange8G = BMI3_ACC_RANGE_8G,  ///< ±8g量程
    eAccelRange16G = BMI3_ACC_RANGE_16G ///< ±16g量程
  } eAccelRange_t;

  /**
   * @enum eGyroRange_t
   * @brief 陀螺仪量程枚举
   * @details 定义陀螺仪的测量量程，单位：dps（度/秒）
   */
  typedef enum {
    eGyroRange125DPS = BMI3_GYR_RANGE_125DPS,   ///< ±125dps量程
    eGyroRange250DPS = BMI3_GYR_RANGE_250DPS,   ///< ±250dps量程
    eGyroRange500DPS = BMI3_GYR_RANGE_500DPS,   ///< ±500dps量程
    eGyroRange1000DPS = BMI3_GYR_RANGE_1000DPS, ///< ±1000dps量程
    eGyroRange2000DPS = BMI3_GYR_RANGE_2000DPS  ///< ±2000dps量程
  } eGyroRange_t;

  /**
   * @enum eAccelODR_t
   * @brief 加速度计输出数据率枚举
   * @details 定义加速度计的采样频率，单位：Hz
   * @note 值以0x00U为基准定义
   */
  typedef enum {
    eAccelODR0_78125Hz = BMI3_ACC_ODR_0_78HZ, ///< 0.78125 Hz
    eAccelODR1_5625Hz = BMI3_ACC_ODR_1_56HZ,  ///< 1.5625 Hz
    eAccelODR3_125Hz = BMI3_ACC_ODR_3_125HZ,  ///< 3.125 Hz
    eAccelODR6_25Hz = BMI3_ACC_ODR_6_25HZ,    ///< 6.25 Hz
    eAccelODR12_5Hz = BMI3_ACC_ODR_12_5HZ,    ///< 12.5 Hz
    eAccelODR25Hz = BMI3_ACC_ODR_25HZ,        ///< 25 Hz
    eAccelODR50Hz = BMI3_ACC_ODR_50HZ,        ///< 50 Hz
    eAccelODR100Hz = BMI3_ACC_ODR_100HZ,      ///< 100 Hz
    eAccelODR200Hz = BMI3_ACC_ODR_200HZ,      ///< 200 Hz
    eAccelODR400Hz = BMI3_ACC_ODR_400HZ,      ///< 400 Hz
    eAccelODR800Hz = BMI3_ACC_ODR_800HZ,      ///< 800 Hz
    eAccelODR1600Hz = BMI3_ACC_ODR_1600HZ,    ///< 1600 Hz
    eAccelODR3200Hz = BMI3_ACC_ODR_3200HZ,    ///< 3200 Hz
    eAccelODR6400Hz = BMI3_ACC_ODR_6400HZ     ///< 6400 Hz
  } eAccelODR_t;

  /**
   * @enum eGyroODR_t
   * @brief 陀螺仪输出数据率枚举
   * @details 定义陀螺仪的采样频率，单位：Hz
   * @note 值以0x00U为基准定义
   */
  typedef enum {
    eGyroODR0_78125Hz = BMI3_GYR_ODR_0_78HZ, ///< 0.78125 Hz
    eGyroODR1_5625Hz = BMI3_GYR_ODR_1_56HZ,  ///< 1.5625 Hz
    eGyroODR3_125Hz = BMI3_GYR_ODR_3_125HZ,  ///< 3.125 Hz
    eGyroODR6_25Hz = BMI3_GYR_ODR_6_25HZ,    ///< 6.25 Hz
    eGyroODR12_5Hz = BMI3_GYR_ODR_12_5HZ,    ///< 12.5 Hz
    eGyroODR25Hz = BMI3_GYR_ODR_25HZ,        ///< 25 Hz
    eGyroODR50Hz = BMI3_GYR_ODR_50HZ,        ///< 50 Hz
    eGyroODR100Hz = BMI3_GYR_ODR_100HZ,      ///< 100 Hz
    eGyroODR200Hz = BMI3_GYR_ODR_200HZ,      ///< 200 Hz
    eGyroODR400Hz = BMI3_GYR_ODR_400HZ,      ///< 400 Hz
    eGyroODR800Hz = BMI3_GYR_ODR_800HZ,      ///< 800 Hz
    eGyroODR1600Hz = BMI3_GYR_ODR_1600HZ,    ///< 1600 Hz
    eGyroODR3200Hz = BMI3_GYR_ODR_3200HZ,    ///< 3200 Hz
    eGyroODR6400Hz = BMI3_GYR_ODR_6400HZ     ///< 6400 Hz
  } eGyroODR_t;

  /**
   * @fn  eInterruptSource_t
   * @brief  Interrupt pin selection
   */
  typedef enum{
    eINT1 = BMI3_INT1, /**<int1 >*/
    eINT2 = BMI3_INT2     /**<int2>*/
  }eInterruptSource_t;

  /**
   * @enum eAxis_t
   * @brief 轴选择掩码
   */
  typedef enum {
    eAxisX = 0x01, ///< X轴
    eAxisY = 0x02, ///< Y轴
    eAxisZ = 0x04, ///< Z轴
    eAxisXYZ = 0x07 ///< XYZ全部
  } eAxis_t;

  /**
   * @struct sSensorData
   * @brief 传感器数据结构
   * @details 用于存储三轴传感器数据（X、Y、Z轴）
   */
  typedef struct {
    float x; ///< X轴数据
    float y; ///< Y轴数据
    float z; ///< Z轴数据
  } sSensorData;
  /**
   * @fn DFRobot_BMI323
   * @brief 构造函数 - I2C接口
   * @param wire TwoWire对象指针，默认&Wire
   */
  DFRobot_BMI323(TwoWire *wire = &Wire);
  ~DFRobot_BMI323();

  /**
   * @fn begin
   * @brief 初始化传感器硬件接口，固定I2C地址0x69
   * @return true 初始化成功
   * @return false 初始化失败
   */
  bool begin(void);


  /**
   * @fn configAccel
   * @brief 配置加速度计
   * @param odr 输出数据率选择（见: eAccelODR_t）
   * @n 可用速率:
   * @n - eAccelODR0_78125Hz:  0.78125 Hz
   * @n - eAccelODR1_5625Hz:   1.5625 Hz
   * @n - eAccelODR3_125Hz:    3.125 Hz
   * @n - eAccelODR6_25Hz:     6.25 Hz
   * @n - eAccelODR12_5Hz:     12.5 Hz
   * @n - eAccelODR25Hz:       25 Hz
   * @n - eAccelODR50Hz:       50 Hz
   * @n - eAccelODR100Hz:      100 Hz
   * @n - eAccelODR200Hz:      200 Hz
   * @n - eAccelODR400Hz:      400 Hz
   * @n - eAccelODR800Hz:      800 Hz
   * @n - eAccelODR1600Hz:     1600 Hz
   * @param range 量程选择（见: eAccelRange_t）
   * @n 可用量程:
   * @n - eAccelRange2G:   ±2g
   * @n - eAccelRange4G:   ±4g
   * @n - eAccelRange8G:   ±8g
   * @n - eAccelRange16G:  ±16g
   * @return true 配置成功
   * @return false 配置失败
   */
  bool configAccel(eAccelODR_t odr, eAccelRange_t range);


  /**
   * @fn configGyro
   * @brief 配置陀螺仪
   * @param odr 输出数据率选择（见: eGyroODR_t）
   * @n 可用速率:
   * @n - eGyroODR0_78125Hz: 0.78125 Hz
   * @n - eGyroODR1_5625Hz:  1.5625 Hz
   * @n - eGyroODR3_125Hz:   3.125 Hz
   * @n - eGyroODR6_25Hz:    6.25 Hz
   * @n - eGyroODR12_5Hz:    12.5 Hz
   * @n - eGyroODR25Hz:      25 Hz
   * @n - eGyroODR50Hz:      50 Hz
   * @n - eGyroODR100Hz:     100 Hz
   * @n - eGyroODR200Hz:     200 Hz
   * @n - eGyroODR400Hz:     400 Hz
   * @n - eGyroODR800Hz:     800 Hz
   * @n - eGyroODR1600Hz:    1600 Hz
   * @n - eGyroODR3200Hz:    3200 Hz
   * @n - eGyroODR6400Hz:    6400 Hz
   * @param range 量程选择（见: eGyroRange_t）
   * @n 可用量程:
   * @n - eGyroRange125DPS:   ±125dps
   * @n - eGyroRange250DPS:   ±250dps
   * @n - eGyroRange500DPS:   ±500dps
   * @n - eGyroRange1000DPS:  ±1000dps
   * @n - eGyroRange2000DPS:  ±2000dps
   * @return true 配置成功
   * @return false 配置失败
   */
  bool configGyro(eGyroODR_t odr, eGyroRange_t range);


  /**
   * @fn readAccelGyro
   * @brief 同时读取加速度计与陀螺仪并返回物理单位
   * @param accel 加速度计输出
   * @param gyro 陀螺仪输出
   * @return true 成功
   */
  bool readAccelGyro(sSensorData *accel, sSensorData *gyro);


  /**
   * @fn getAccelGyroData
   * @brief 同步读取陀螺仪与加速度计（前3个为陀螺仪，后3个为加速度计）
   * @param data int16_t[6]缓存
   * @return int8_t BMI3_OK表示成功
   */
  int8_t getAccelGyroData(int16_t *data);



  /**
   * @fn enableStepCounterInterrupt
   * @brief 启用计步器中断功能
   * @details 配置计步器功能并映射到指定的中断引脚，当检测到步数变化时会触发中断
   * @param pin 绑定的中断引脚（eINT1 或 eINT2）
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableStepCounterInterrupt(eInterruptSource_t pin);

  /**
   * @fn readStepCounter
   * @brief 读取计步器数据
   * @param stepVal 步数输出指针，读取的步数将写入此指针指向的内存
   * @return int8_t BMI3_OK表示成功，其它值表示失败
   */
  int8_t readStepCounter(uint16_t *stepVal);

  /**
   * @fn getInterruptStatus
   * @brief 获取中断状态
   * @return uint16_t 中断状态寄存器值
   */
  uint16_t getInterruptStatus(void);

  /**
   * @fn enableAnyMotionInterrupt
   * @brief 配置任意运动阈值中断（使用官方结构体参数）
   * @param config 任意运动配置结构体（见 bmi3_any_motion_config）
   * @n 参数说明：
   * @n - slope_thres: 加速度斜率阈值，范围 0-4095，单位 1.953mg/LSB（官方示例：9 ≈ 17.6mg）
   * @n - hysteresis: 迟滞值，范围 0-1023，单位 1.953mg/LSB（官方示例：5 ≈ 9.8mg）
   * @n - duration: 持续时间，范围 0-8191，单位 20ms（官方示例：9 = 180ms）
   * @n - acc_ref_up: 加速度参考更新模式，0=OnEvent, 1=Always（官方示例：1）
   * @n - wait_time: 等待时间，范围 0-7，单位 20ms（官方示例：4-5 = 80-100ms）
   * @param pin 绑定的中断引脚
   * @param axisMask 轴选择掩码（默认：eAxisXYZ）
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableAnyMotionInterrupt(const struct bmi3_any_motion_config &config,
                                eInterruptSource_t pin, uint8_t axisMask = eAxisXYZ);

  /**
   * @fn enableNoMotionInterrupt
   * @brief 配置静止阈值中断（使用官方结构体参数）
   * @param config 静止检测配置结构体（见 bmi3_no_motion_config）
   * @n 参数说明：
   * @n - slope_thres: 加速度斜率阈值，范围 0-4095，单位 1.953mg/LSB（官方示例：9 ≈ 17.6mg）
   * @n - hysteresis: 迟滞值，范围 0-1023，单位 1.953mg/LSB（官方示例：5 ≈ 9.8mg）
   * @n - duration: 持续时间，范围 0-8191，单位 20ms（官方示例：9 = 180ms）
   * @n - acc_ref_up: 加速度参考更新模式，0=OnEvent, 1=Always（官方示例：1）
   * @n - wait_time: 等待时间，范围 0-7，单位 20ms（官方示例：5 = 100ms）
   * @param pin 绑定的中断引脚
   * @param axisMask 轴选择掩码（默认：eAxisXYZ）
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableNoMotionInterrupt(const struct bmi3_no_motion_config &config,
                               eInterruptSource_t pin, uint8_t axisMask = eAxisXYZ);

  /**
   * @fn enableSigMotionInterrupt
   * @brief 配置显著运动检测中断（使用官方结构体参数）
   * @param config 显著运动配置结构体（见 bmi3_sig_motion_config）
   * @n 参数说明：
   * @n - block_size: 检测段大小，范围 0-65535（官方示例：200）
   * @n - peak_2_peak_min: 峰峰值加速度最小值，范围 0-1023（官方示例：30）
   * @n - peak_2_peak_max: 峰峰值加速度最大值，范围 0-1023（官方示例：30）
   * @n - mcr_min: 每秒平均交叉数最小值，范围 0-62（官方示例：0x10 = 16）
   * @n - mcr_max: 每秒平均交叉数最大值，范围 0-62（官方示例：0x10 = 16）
   * @param pin 绑定的中断引脚
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableSigMotionInterrupt(const struct bmi3_sig_motion_config &config,
                                 eInterruptSource_t pin);

  /**
   * @fn enableFlatInterrupt
   * @brief 配置平面检测中断（使用官方结构体参数）
   * @param config 平面检测配置结构体（见 bmi3_flat_config）
   * @n 参数说明：
   * @n - theta: 最大允许倾斜角度，范围 0-63，角度计算为 64 * (tan(angle)^2)（官方示例：9）
   * @n - blocking: 阻塞模式，0=MODE_0(禁用), 1=MODE_1(>1.5g), 2=MODE_2(>1.5g或斜率>一半阈值), 3=MODE_3(>1.5g或斜率>阈值)（官方示例：3）
   * @n - hold_time: 设备保持平面状态的最小时长，范围 0-255，单位 20ms（官方示例：50 = 1000ms）
   * @n - hysteresis: 平面检测的迟滞角度，范围 0-255（官方示例：9）
   * @n - slope_thres: 连续加速度样本之间的最小斜率，范围 0-255（官方示例：0xCD = 205）
   * @param pin 绑定的中断引脚
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableFlatInterrupt(const struct bmi3_flat_config &config, eInterruptSource_t pin);

  /**
   * @fn enableOrientationInterrupt
   * @brief 配置方向检测中断（使用官方结构体参数）
   * @param config 方向检测配置结构体（见 bmi3_orientation_config）
   * @n 参数说明：
   * @n - ud_en: 是否检测翻转（面朝上/下），0=禁用，1=启用（官方示例：1）
   * @n - hold_time: 方向改变判定所需持续时间，范围 0-255，单位 20ms（官方示例：4 = 80ms）
   * @n - hysteresis: 方向检测的迟滞，范围 0-255（官方示例：5）
   * @n - theta: 允许的最大倾斜角，范围 0-63，角度=64*(tan(angle)^2)（官方示例：16）
   * @n - mode: 方向检测模式，0/3=对称，1=高不对称，2=低不对称（官方示例：1）
   * @n - slope_thres: 防止剧烈运动导致误判的斜率阈值，范围 0-255（官方示例：30）
   * @n - blocking: 阻塞模式，0-3（官方示例：3）
   * @param pin 绑定的中断引脚
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableOrientationInterrupt(const struct bmi3_orientation_config &config,
                                  eInterruptSource_t pin);

  /**
   * @fn readOrientation
   * @brief 读取方向检测输出
   * @param portraitLandscape Portrait/Landscape状态输出指针，可为NULL
   * @param faceUpDown Face up/down状态输出指针，可为NULL
   * @return true 读取成功
   * @return false 读取失败或特性未启用
   */
  bool readOrientation(uint8_t *portraitLandscape, uint8_t *faceUpDown);

  /**
   * @fn enableTapInterrupt
   * @brief 配置轻击检测中断（使用官方结构体参数）
   * @param config 轻击检测配置结构体（见 bmi3_tap_detector_config）
   * @n 关键参数（参考 tap.c）：
   * @n - axis_sel: 选择用于轻击检测的轴（0=X,1=Y,2=Z）
   * @n - mode: 检测模式（0=敏感、1=正常、2=鲁棒）
   * @n - tap_peak_thres / tap_shock_settling_dur 等用于判定敲击的时序/幅度阈值
   * @param pin 绑定的中断引脚
   * @param enableSingle 是否启用单击检测（默认 true）
   * @param enableDouble 是否启用双击检测（默认 true）
   * @param enableTriple 是否启用三击检测（默认 true）
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableTapInterrupt(const struct bmi3_tap_detector_config &config,
                          eInterruptSource_t pin, bool enableSingle = true,
                          bool enableDouble = true, bool enableTriple = true);

  /**
   * @fn readTapStatus
   * @brief 读取轻击检测状态（单/双/三击）
   * @param tapMask 输出掩码（可组合 BMI3_TAP_DET_STATUS_SINGLE/DOUBLE/TRIPLE）
   * @return true 读取成功
   * @return false 读取失败
   */
  bool readTapStatus(uint8_t *tapMask);

  /**
   * @fn enableTiltInterrupt
   * @brief 配置倾斜检测中断（使用官方结构体参数）
   * @param config 倾斜检测配置结构体（见 bmi3_tilt_config）
   * @n 关键参数（参考 tilt.c）：
   * @n - segment_size: 用于平均参考向量的时间窗口，范围 0-255
   * @n - min_tilt_angle: 需要超过的最小倾斜角，范围 0-255，角度=256*cos(angle)
   * @n - beta_acc_mean: 低通平均系数，范围 0-65535
   * @param pin 绑定的中断引脚
   * @return true 配置成功
   * @return false 配置失败
   */
  bool enableTiltInterrupt(const struct bmi3_tilt_config &config, eInterruptSource_t pin);


private:
  /** I2C相关 */
  uint8_t _i2cAddr = 0x69; ///< I2C设备地址固定0x69
  TwoWire *_wire;   ///< TwoWire对象指针

  /** 设备结构体 */
  struct bmi3_dev _dev; ///< BMI3设备结构体

  /** 内部标志 */
  bool _initialized; ///< 初始化标志

  /** 当前加速度计量程（用于单位转换） */
  float _accelRange; ///< 加速度计量程（单位：g）

  /** 当前陀螺仪量程（用于单位转换） */
  float _gyroRange; ///< 陀螺仪量程（单位：dps）

  /** 中断引脚配置状态 */
  bool _intPinConfigured[2];

  /** 中断映射缓存 */
  struct bmi3_map_int _intMapConfig;

  /** 特性开关缓存 */
  struct bmi3_feature_enable _featureEnable;

  /**
   * @fn _initInterface
   * @brief 初始化I2C接口
   * @return int8_t 0表示成功，负数表示失败
   */
  int8_t _initInterface(void);

  /**
   * @fn _i2cRead
   * @brief I2C读取函数
   * @param reg_addr 寄存器地址
   * @param reg_data 数据缓冲区
   * @param len 数据长度
   * @param intf_ptr 接口指针
   * @return int8_t 0表示成功，负数表示失败
   */
  int8_t _i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                  void *intf_ptr);

  /**
   * @fn _i2cWrite
   * @brief I2C写入函数
   * @param reg_addr 寄存器地址
   * @param reg_data 数据缓冲区
   * @param len 数据长度
   * @param intf_ptr 接口指针
   * @return int8_t 0表示成功，负数表示失败
   */
  int8_t _i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                   void *intf_ptr);

  /**
   * @fn _delayUs
   * @brief 微秒延时函数
   * @param period 延时时间（微秒）
   * @param intf_ptr 接口指针
   */
  void _delayUs(uint32_t period, void *intf_ptr);

  /** 静态回调函数（用于底层驱动） */
  static int8_t _i2cReadCallback(uint8_t reg_addr, uint8_t *reg_data,
                                 uint32_t len, void *intf_ptr);
  static int8_t _i2cWriteCallback(uint8_t reg_addr, const uint8_t *reg_data,
                                  uint32_t len, void *intf_ptr);
  static void _delayUsCallback(uint32_t period, void *intf_ptr);

  /**
   * @fn _lsbToG
   * @brief 将LSB值转换为G值
   * @param val 原始LSB值
   * @param g_range 量程（g）
   * @return float G值
   */
  float _lsbToG(int16_t val, float g_range);

  /**
   * @fn _lsbToDps
   * @brief 将LSB值转换为度/秒值
   * @param val 原始LSB值
   * @param dps_range 量程（dps）
   * @return float 度/秒值
   */
  float _lsbToDps(int16_t val, float dps_range);

  bool _readVector(uint8_t sensor_type, sSensorData *data, float range,
                   bool isAccel);
  bool _configureMotionInterrupt(bool anyMotion,
                                 const struct bmi3_any_motion_config *anyConfig,
                                 const struct bmi3_no_motion_config *noConfig,
                                 eInterruptSource_t pin, uint8_t axisMask);
  bool _configureIntPin(eInterruptSource_t pin);
  uint8_t _encodeIntPin(eInterruptSource_t pin) const;
  uint16_t _mgToSlope(float threshold_mg) const;
  uint16_t _msToDuration(uint16_t duration_ms) const;
  uint8_t _msToWait(uint16_t duration_ms) const;
  bool _applyFeatureEnable(void);

  /**
   * @fn getAccelData
   * @brief 内部函数：获取原始加速度计LSB（被getAccelGyroData使用）
   * @param data int16_t[3]缓存
   * @return int8_t BMI3_OK表示成功
   */
  int8_t getAccelData(int16_t *data);

  /**
   * @fn getGyroData
   * @brief 内部函数：获取原始陀螺仪LSB（被getAccelGyroData使用）
   * @param data int16_t[3]缓存
   * @return int8_t BMI3_OK表示成功
   */
  int8_t getGyroData(int16_t *data);
};

#endif
