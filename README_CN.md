# DFRobot_BMP58X

* [English Version](./README.md)

这是一个BMP58X的库，功能是读取温度和压力。BMP(585/581)是一款基于可靠传感原理的压力和温度测量数字传感器。

### SEN0664
<p align="center">
  <img src="./resources/images/SEN0664 (3).png" width="45%">
  <img src="./resources/images/SEN0664 (4).png" width="45%">
</p>

### SEN0665
<p align="center">
  <img src="./resources/images/SEN0665 (1).png" width="45%">
  <img src="./resources/images/SEN0665 (2).png" width="45%">
</p>

### SEN0666
<p align="center">
  <img src="./resources/images/SEN0666 (3).png" width="45%">
  <img src="./resources/images/SEN0666 (4).png" width="45%">
</p>

### SEN0667
<p align="center">
  <img src="./resources/images/SEN0667 (1).png" width="45%">
  <img src="./resources/images/SEN0667 (2).png" width="45%">
</p>


## 产品链接(https://www.dfrobot.com)
    SKU: SEN0664/SEN0665/SEN0666/SEN0667

## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

* 该库支持BMP585/581传感器。
* 该库支持读取温度和压力。
* 该库支持设置传感器的工作模式。
* 该库支持设置传感器的输出数据率。
* 该库支持设置传感器的过采样率。
* 该库支持设置传感器的IIR滤波器系数。
* 该库支持设置传感器的FIFO操作。
* 该库支持设置传感器的中断行为。
* 该库支持设置传感器的压力超限检测。
* 该库支持读取传感器的中断状态。

## 库安装

使用此库前，请首先安装依赖库 DFRobot_RTU，让后再下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```C++
/**
 * @fn begin
 * @brief 初始化传感器硬件接口
 * @return 初始化成功返回true，失败返回false
 */
bool begin(void);

/**
 * @fn setODR
 * @brief 配置传感器输出数据率(ODR)
 * @param odr 输出数据率选择(见: eOdr_t)
 * @n 可用速率:
 * @n - eOdr240Hz:    输出速率240 Hz
 * @n - eOdr218_5Hz:  输出速率218.5 Hz
 * @n - eOdr199_1Hz:  输出速率199.1 Hz
 * @n - eOdr179_2Hz:  输出速率179.2 Hz
 * @n - eOdr160Hz:    输出速率160 Hz
 * @n - eOdr149_3Hz:  输出速率149.3 Hz
 * @n - eOdr140Hz:    输出速率140 Hz
 * @n - eOdr129_8Hz:  输出速率129.8 Hz
 * @n - eOdr120Hz:    输出速率120 Hz
 * @n - eOdr110_1Hz:  输出速率110.1 Hz
 * @n - eOdr100_2Hz:  输出速率100.2 Hz
 * @n - eOdr89_6Hz:   输出速率89.6 Hz
 * @n - eOdr80Hz:     输出速率80 Hz
 * @n - eOdr70Hz:     输出速率70 Hz
 * @n - eOdr60Hz:     输出速率60 Hz
 * @n - eOdr50Hz:     输出速率50 Hz
 * @n - eOdr45Hz:     输出速率45 Hz
 * @n - eOdr40Hz:     输出速率40 Hz
 * @n - eOdr35Hz:     输出速率35 Hz
 * @n - eOdr30Hz:     输出速率30 Hz
 * @n - eOdr25Hz:     输出速率25 Hz
 * @n - eOdr20Hz:     输出速率20 Hz
 * @n - eOdr15Hz:     输出速率15 Hz
 * @n - eOdr10Hz:     输出速率10 Hz
 * @n - eOdr5Hz:      输出速率5 Hz
 * @n - eOdr4Hz:      输出速率4 Hz
 * @n - eOdr3Hz:      输出速率3 Hz
 * @n - eOdr2Hz:      输出速率2 Hz
 * @n - eOdr1Hz:      输出速率1 Hz
 * @n - eOdr0_5Hz:    输出速率0.5 Hz
 * @n - eOdr0_250Hz:  输出速率0.250 Hz
 * @n - eOdr0_125Hz:  输出速率0.125 Hz
 * @return 成功返回0，错误返回1
 */
uint8_t setODR(eODR_t odr);

/**
 * @fn setOSR
 * @brief 设置温度和压力的过采样率
 * @param osrTemp 温度过采样(见: eOverSampling_t)
 * @param osrPress 压力过采样(见: eOverSampling_t)
 * @n 支持的值:
 * @n - eOverSampling1:   1倍过采样
 * @n - eOverSampling2:   2倍过采样
 * @n - eOverSampling4:   4倍过采样
 * @n - eOverSampling8:   8倍过采样
 * @n - eOverSampling16:  16倍过采样
 * @n - eOverSampling32:  32倍过采样
 * @n - eOverSampling64:  64倍过采样
 * @n - eOverSampling128: 128倍过采样
 * @return 成功返回0，错误返回1
 */
uint8_t setOSR(eOverSampling_t osrTemp, eOverSampling_t osrPress);

/**
 * @fn setMeasureMode
 * @brief 配置传感器电源/测量模式
 * @param mode 操作模式(见: eMeasureMode_t)
 * @n 可用模式:
 * @n - eSleep:         睡眠模式
 * @n - eNormal:        正常测量模式
 * @n - eSingleShot:    单次测量
 * @n - eContinuous:    连续测量
 * @n - eDeepSleep:     深度睡眠模式
 * @return 成功返回0，错误返回1
 */
uint8_t setMeasureMode(eMeasureMode_t mode);

/**
 * @fn reset
 * @brief 执行传感器软件复位
 * @return 成功返回0，错误返回1
 */
uint8_t reset(void);

/**
 * @fn readTempC
 * @brief 读取校准后的温度数据
 * @return 温度值(摄氏度)
 */
float readTempC(void);

/**
 * @fn readPressPa
 * @brief 读取校准后的压力数据
 * @return 压力值(帕斯卡)
 */
float readPressPa(void);

/**
 * @fn readAltitudeM
 * @brief 根据压力读数计算海拔高度
 * @note 使用公式:
 * @n altitude = (1 - (P/101325)^0.190284) * 44307.7
 * @n 其中P = 当前压力值(Pa)
 * @return 海拔高度(米)
 */
float readAltitudeM(void);

/**
 * @fn configIIR
 * @brief 配置IIR滤波器系数
 * @param iirTemp  温度IIR滤波器(见: eIIRFilter_t)
 * @param iirPress 压力IIR滤波器(见: eIIRFilter_t)
 * @n 可用系数:
 * @n - eFilterBypass:   旁路滤波器
 * @n - eFilter1:        1阶滤波器
 * @n - eFilter3:        3阶滤波器
 * @n - eFilter7:        7阶滤波器
 * @n - eFilter15:       15阶滤波器
 * @n - eFilter31:       31阶滤波器
 * @n - eFilter63:       63阶滤波器
 * @n - eFilter127:      127阶滤波器
 * @return 成功返回0，错误返回1
 */
uint8_t configIIR(eIIRFilter_t iirTemp, eIIRFilter_t iirPress);

/**
 * @fn configFIFO
 * @brief 配置FIFO操作参数
 * @param dataSel 数据帧类型(见: eFIFODataSel_t)
 * @n 可用类型:
 * @n - eFIFODisable:           FIFO禁用
 * @n - eFIFOTempData:          仅温度数据
 * @n - eFIFOPressData:         仅压力数据
 * @n - eFIFOPressAndTempData:  压力和温度数据
 *
 * @param downSampling 下采样率(见: eFIFODownSampling_t)
 * @n 可用比率:
 * @n - eNoDownSampling:    无下采样
 * @n - eDownSampling2:     2倍下采样
 * @n - eDownSampling4:     4倍下采样
 * @n - eDownSampling8:     8倍下采样
 * @n - eDownSampling16:    16倍下采样
 * @n - eDownSampling32:    32倍下采样
 * @n - eDownSampling64:    64倍下采样
 * @n - eDownSampling128:   128倍下采样
 *
 * @param mode FIFO操作模式(见: eFIFOWorkMode_t)
 * @n 可用模式:
 * @n - eFIFOOverwriteMode:  连续数据流模式
 * @n - eFIFOFullStopMode:   FIFO满时停止
 *
 * @param threshold FIFO触发阈值(0=禁用, 1-31=帧数)
 * @n - 0x0F: 15帧。这是PT模式下的最大设置。最高有效位被忽略。
 * @n - 0x1F: 31帧。这是P或T模式下的最大设置。
 * @return 成功返回0，错误返回1
 */
uint8_t configFIFO(eFIFODataSel_t dataSel, eFIFODownSampling_t downSampling, eFIFOWorkMode_t mode, uint8_t threshold);

/**
 * @fn getFIFOCount
 * @brief 获取FIFO中当前的帧数
 * @return 存储的数据帧数(0-31)
 */
uint8_t getFIFOCount(void);

/**
 * @fn getFIFOData
 * @brief 从FIFO读取所有数据
 * @return sFIFOData_t 包含压力和温度数据的结构体
 * @n - len: 存储的数据帧数(0-31)
 * @n - pressure: 压力值数组
 * @n - temperature: 温度值数组
 */
sFIFOData_t getFIFOData(void);

/**
 * @fn configInterrupt
 * @brief 配置中断行为
 * @param intMode 触发模式(见: eIntMode_t)
 * @n 可用模式:
 * @n - eIntModePulsed: 脉冲模式
 * @n - eIntModeLatched: 锁存模式
 *
 * @param intPol 信号极性(见: eIntPolarity_t)
 * @n 可用极性:
 * @n - eIntLowActive:  低电平有效
 * @n - eIntHighActive: 高电平有效
 *
 * @param intOd 输出驱动类型(见: eIntOutputMode_t)
 * @n 可用类型:
 * @n - eIntPushPull: 推挽输出
 * @n - eIntOpenDrain: 开漏输出
 *
 * @return 成功返回0，错误返回1
 */
uint8_t configInterrupt(eIntMode_t mode, eIntPolarity_t pol, eIntOutputMode_t outputMode);

/**
 * @fn setIntSource
 * @brief 启用特定的中断源
 * @param source 触发位掩码(见: eIntSource_t)
 * @n 可用源:
 * @n - eIntDataReady:    数据就绪中断
 * @n - eIntFIFOFull:    FIFO满中断
 * @n - eIntFIFOThres:   FIFO阈值中断
 * @n - eIntPressureOOR: 压力超出范围中断
 * @details 可以使用按位或(|)组合多个中断源。
 *          示例: 同时启用数据就绪和FIFO满中断:
 *          @code
 *          setIntSource(bmp58x.eIntDataReady | bmp58x.eIntFIFOFull);
 *          @endcode
 * @return 成功返回0，错误返回1
 */
uint8_t setIntSource(uint8_t source);

/**
 * @fn getIntStatus
 * @brief 读取当前中断状态标志
 * @return uint16_t 活动中断的位掩码
 * @n 可能的标志:
 * @n - eIntStatusDataReady: 数据就绪(0x01)
 * @n - eIntStatusFIFOFull: FIFO已满 (0x02)
 * @n - eIntStatusFIFOThres: FIFO达到阈值(0x04)
 * @n - eIntStatusPressureOOR: 压力超出范围(0x08)
 * @n - eIntStatusResetComplete: 复位完成(0x10)
 */
uint16_t getIntStatus(void);

/**
 * @fn setPressOOR
 * @brief 配置压力超出范围检测
 * @param oor 阈值压力值(0x00000-0x1FFFF)
 * @param range 迟滞范围(0-255)
 * @n oor - range < 压力 < oor + range
 * @param cntLimit 触发持续计数
 * @n 可用持续设置:
 * @n - eOORCountLimit1:  1次计数
 * @n - eOORCountLimit3:  3次计数
 * @n - eOORCountLimit7:  7次计数
 * @n - eOORCountLimit15: 15次计数
 * @return 成功返回0，错误返回1
 */
uint8_t setPressOOR(uint32_t oor, uint8_t range, eOORCountLimit_t cntLimit);

/**
 * @fn calibratedAbsoluteDifference
 * @brief 使用给定的当前海拔高度作为参考值，消除后续压力和海拔数据的绝对差值
 * @param altitude 当前海拔高度
 * @return 布尔值，表示参考值是否设置成功
 * @retval True 表示参考值设置成功
 * @retval False 表示设置参考值失败
 */
bool calibratedAbsoluteDifference(float altitude);

/**
 * @fn setBaud
 * @brief 设置UART通信波特率
 * @details 使用指定的波特率枚举值配置串行通信速度。
 *          该函数初始化必要的硬件寄存器以实现所需的数据传输速率。
 * @param baud eBaud枚举值，指定所需的波特率。
 *             如果未显式设置，默认为e9600。
 * @note 实际硬件配置可能因微控制器型号而异。
 *       该函数假设使用标准时钟频率；如果使用非默认系统时钟配置，请调整时钟设置。
 * @warning 在通信过程中更改波特率可能导致数据丢失或通信错误（如果两个设备未同步）。
 * @see eBaud可用的波特率选项：
 *      - e2400: 2400比特/秒
 *      - e4800: 4800比特/秒  
 *      - e9600: 9600比特/秒（默认值）
 *      - e14400: 14400比特/秒
 *      - e19200: 19200比特/秒
 *      - e38400: 38400比特/秒
 *      - e57600: 57600比特/秒
 *      - e115200: 115200比特/秒
 */
void setBaud(eBaud baud);

```

## 兼容性

| MCU                | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------------------ |:---------:|:----------:|:--------:| ------- |
| Arduino uno        |     √     |            |          |         |
| FireBeetle esp32   |     √     |            |          |         |
| FireBeetle esp8266 |     √     |            |          |         |
| FireBeetle m0      |     √     |            |          |         |
| Leonardo           |     √     |            |          |         |
| Microbit           |     √     |            |          |         |
| Arduino MEGA2560   |     √     |            |          |         |

## 历史

- Data 2025-09-22
- Version V1.0.0

## 创作者

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
