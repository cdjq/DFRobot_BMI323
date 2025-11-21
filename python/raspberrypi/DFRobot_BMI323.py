# -*- coding: utf-8 -*
'''!
  @file  DFRobot_BMP58X.py
  @brief  Define the infrastructure of DFRobot_BMP58X class
  @n      This is a pressure and temperature sensor that can be controlled through I2C/SPI/UART ports.
  @n      BMP (581/585) has functions such as temperature compensation, data oversampling, IIR filtering, etc
  @n      These features improve the accuracy of data collected by BMP (581/585) sensors.
  @n      BMP (581/585) also has a FIFO data buffer, greatly improving its availability.
  @n      Similarly, BMP (581/585) has an interrupt pin that can be used in an energy-efficient manner without using software algorithms.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version     V1.0.0
  @date        2025-06-06
  @url         https://github.com/DFRobot/DFRobot_BMP58X
'''
  
import struct
from ctypes import *
from DFRobot_RTU import *
import sys
import time

import smbus
import spidev
import RPi.GPIO as GPIO

import logging
from ctypes import *

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


logger = logging.getLogger()
# logger.setLevel(logging.INFO)   # Display all print information
# If you don’t want to display too many prints, only print errors, please use this option
logger.setLevel(logging.FATAL)
ph = logging.StreamHandler()
formatter = logging.Formatter(
    "%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter)
logger.addHandler(ph)


class DFRobot_BMP58X(object):
    # Register address definitions
    REG_DATA_LEN_MAX = 0x04                     # Maximum data length
    REG_I_CHIP_ID = 0x01                        # Chip ID register
    REG_I_REV_ID = 0x02                         # Revision ID register
    REG_I_CHIP_STATUS = 0x11                    # Chip status register
    REG_H_DRIVE_CONFIG = 0x13                   # Drive configuration register
    REG_H_INT_CONFIG = 0x14                     # Interrupt configuration register
    REG_H_INT_SOURCE = 0x15                     # Interrupt source selection register
    REG_H_FIFO_CONFIG = 0x16                    # FIFO configuration register
    REG_I_FIFO_COUNT = 0x17                     # FIFO count register
    REG_H_FIFO_SEL = 0x18                       # FIFO selection register
    REG_I_TEMP_DATA_XLSB = 0x1D                 # Temperature data XLSB register
    REG_I_TEMP_DATA_LSB = 0x1E                  # Temperature data LSB register
    REG_I_TEMP_DATA_MSB = 0x1F                  # Temperature data MSB register
    REG_I_PRESS_DATA_XLSB = 0x20                # Pressure data XLSB register
    REG_I_PRESS_DATA_LSB = 0x21                 # Pressure data LSB register
    REG_I_PRESS_DATA_MSB = 0x22                 # Pressure data MSB register
    REG_I_INT_STATUS = 0x27                     # Interrupt status register
    REG_I_STATUS = 0x28                         # Status register
    REG_I_FIFO_DATA = 0x29                      # FIFO data register
    REG_H_NVM_ADDR = 0x2B                       # NVM address register
    REG_H_NVM_DATA_LSB = 0x2C                   # NVM data LSB register
    REG_H_NVM_DATA_MSB = 0x2D                   # NVM data MSB register
    REG_H_DSP_CONFIG = 0x30                     # DSP configuration register
    REG_H_DSP_IIR = 0x31                        # DSP IIR configuration register
    REG_H_OOR_THR_P_LSB = 0x32                  # Out-of-range pressure threshold LSB register
    REG_H_OOR_THR_P_MSB = 0x33                  # Out-of-range pressure threshold MSB register
    REG_H_OOR_RANGE = 0x34                      # Out-of-range pressure range register
    REG_H_OOR_CONFIG = 0x35                     # Out-of-range configuration register
    REG_H_OSR_CONFIG = 0x36                     # Oversampling configuration register
    REG_H_ODR_CONFIG = 0x37                     # Output data rate configuration register
    REG_I_OSR_EFF = 0x38                        # Effective oversampling register
    REG_H_CMD = 0x7E                            # Command register

    REG_I_MODBUS_PID = 0x00                     # Modbus PID register
    REG_I_MODBUS_VID = 0x01                     # Modbus VID register
    REG_H_MODBUS_BAUD = 0x01                    # Modbus baud rate register
    OFFSET_REG = 0x0006                         # Register offset

    BMP581_CHIP_ID = 0x50                       # BMP581 chip ID
    BMP585_CHIP_ID = 0x51                       # BMP585 chip ID

    # /*! @name Soft reset command */
    SOFT_RESET_CMD = 0xB6                       # Soft reset command

    # /** Bit position and width macro definitions */
    # // REG_INT_CONFIG (0x14)
    INT_MODE_POS = 0                            # Interrupt mode position
    INT_MODE_WIDTH = 1                          # Interrupt mode width
    INT_POL_POS = 1                             # Interrupt polarity position
    INT_POL_WIDTH = 1                           # Interrupt polarity width
    INT_OD_POS = 2                              # Interrupt open-drain position
    INT_OD_WIDTH = 1                            # Interrupt open-drain width
    INT_EN_POS = 3                              # Interrupt enable position
    INT_EN_WIDTH = 1                            # Interrupt enable width
    PAD_INT_DRV_POS = 4                         # PAD interrupt drive position
    PAD_INT_DRV_WIDTH = 4                       # PAD interrupt drive width

    # // REG_INT_SRC_SEL (0x15)
    DRDY_DATA_REG_EN_POS = 0                    # Data ready register enable position
    DRDY_DATA_REG_EN_WIDTH = 1                  # Data ready register enable width
    FIFO_FULL_EN_POS = 1                        # FIFO full enable position
    FIFO_FULL_EN_WIDTH = 1                      # FIFO full enable width
    FIFO_THS_EN_POS = 2                         # FIFO threshold enable position
    FIFO_THS_EN_WIDTH = 1                       # FIFO threshold enable width
    OOR_P_EN_POS = 3                            # Out-of-range pressure enable position
    OOR_P_EN_WIDTH = 1                          # Out-of-range pressure enable width

    # // REG_FIFO_CONFIG (0x16)
    FIFO_THRESHOLD_POS = 0                      # FIFO threshold position
    FIFO_THRESHOLD_WIDTH = 5                    # FIFO threshold width
    FIFO_MODE_POS = 5                           # FIFO mode position
    FIFO_MODE_WIDTH = 1                         # FIFO mode width

    # // REG_FIFO_COUNT (0x17)
    FIFO_COUNT_POS = 0                          # FIFO count position
    FIFO_COUNT_WIDTH = 6                        # FIFO count width

    # // REG_FIFO_SEL_CONFIG (0x18)
    FIFO_FRAME_SEL_POS = 0                      # FIFO frame selection position
    FIFO_FRAME_SEL_WIDTH = 2                    # FIFO frame selection width
    FIFO_DEC_SEL_POS = 2                        # FIFO decimation selection position
    FIFO_DEC_SEL_WIDTH = 3                      # FIFO decimation selection width

    # // REG_NVM_ADDRESS (0x2B)
    NVM_ROW_ADDRESS_POS = 0                     # NVM row address position
    NVM_ROW_ADDRESS_WIDTH = 6                   # NVM row address width
    NVM_PROG_EN_POS = 6                         # NVM programming enable position
    NVM_PROG_EN_WIDTH = 1                       # NVM programming enable width

    # // REG_DSP_CONFIG (0x30)
    IIR_FLUSH_FORCED_EN_POS = 2                 # IIR forced flush enable position
    IIR_FLUSH_FORCED_EN_WIDTH = 1               # IIR forced flush enable width
    SHDW_SEL_IIR_T_POS = 3                      # Temperature IIR shadow selection position
    SHDW_SEL_IIR_T_WIDTH = 1                    # Temperature IIR shadow selection width
    FIFO_SEL_IIR_T_POS = 4                      # Temperature IIR FIFO selection position
    FIFO_SEL_IIR_T_WIDTH = 1                    # Temperature IIR FIFO selection width
    SHDW_SEL_IIR_P_POS = 5                      # Pressure IIR shadow selection position
    SHDW_SEL_IIR_P_WIDTH = 1                    # Pressure IIR shadow selection width
    FIFO_SEL_IIR_P_POS = 6                      # Pressure IIR FIFO selection position
    FIFO_SEL_IIR_P_WIDTH = 1                    # Pressure IIR FIFO selection width
    OOR_SEL_IIR_P_POS = 7                       # Out-of-range pressure IIR selection position
    OOR_SEL_IIR_P_WIDTH = 1                     # Out-of-range pressure IIR selection width

    # // REG_DSP_IIR_CONFIG (0x31)
    SET_IIR_T_POS = 0                           # Temperature IIR setting position
    SET_IIR_T_WIDTH = 3                         # Temperature IIR setting width
    SET_IIR_P_POS = 3                           # Pressure IIR setting position
    SET_IIR_P_WIDTH = 3                         # Pressure IIR setting width

    # // REG_OOR_CONFIG (0x35)
    OOR_THR_P_16_POS = 0                        # Out-of-range pressure threshold 16-bit position
    OOR_THR_P_16_WIDTH = 1                      # Out-of-range pressure threshold 16-bit width
    CNT_LIM_POS = 6                             # Count limit position
    CNT_LIM_WIDTH = 2                           # Count limit width

    # // REG_OSR_CONFIG (0x36)
    OSR_T_POS = 0                               # Temperature oversampling position
    OSR_T_WIDTH = 3                             # Temperature oversampling width
    OSR_P_POS = 3                               # Pressure oversampling position
    OSR_P_WIDTH = 3                             # Pressure oversampling width
    PRESS_EN_POS = 6                            # Pressure enable position
    PRESS_EN_WIDTH = 1                          # Pressure enable width

    # // REG_ODR_CONFIG (0x37)
    PWR_MODE_POS = 0                            # Power mode position
    PWR_MODE_WIDTH = 2                          # Power mode width
    ODR_POS = 2                                 # Output data rate position
    ODR_WIDTH = 5                               # Output data rate width
    DEEP_DIS_POS = 7                            # Deep sleep disable position
    DEEP_DIS_WIDTH = 1                          # Deep sleep disable width

    # // REG_EFF_OSR_CONFIG (0x38)
    OSR_T_EFF_POS = 0                           # Effective temperature oversampling position
    OSR_T_EFF_WIDTH = 3                         # Effective temperature oversampling width
    OSR_P_EFF_POS = 3                           # Effective pressure oversampling position
    OSR_P_EFF_WIDTH = 3                         # Effective pressure oversampling width
    ODR_IS_VALID_POS = 7                        # ODR validity position
    ODR_IS_VALID_WIDTH = 1                      # ODR validity width

    STANDARD_SEA_LEVEL_PRESSURE_PA = 101325     # Standard sea level pressure, unit: Pa

    # Constant definitions
    DISABLE = 0                                 # Disable
    ENABLE = 1                                  # Enable

    DEEP_ENABLE = 0                             # Deep sleep enable
    DEEP_DISABLE = 1                            # Deep sleep disable

    # Output data rate options
    ODR_240_HZ = 0x00                           # 240 Hz
    ODR_218_5_HZ = 0x01                         # 218.5 Hz
    ODR_199_1_HZ = 0x02                         # 199.1 Hz
    ODR_179_2_HZ = 0x03                         # 179.2 Hz
    ODR_160_HZ = 0x04                           # 160 Hz
    ODR_149_3_HZ = 0x05                         # 149.3 Hz
    ODR_140_HZ = 0x06                           # 140 Hz
    ODR_129_8_HZ = 0x07                         # 129.8 Hz
    ODR_120_HZ = 0x08                           # 120 Hz
    ODR_110_1_HZ = 0x09                         # 110.1 Hz
    ODR_100_2_HZ = 0x0A                         # 100.2 Hz
    ODR_89_6_HZ = 0x0B                          # 89.6 Hz
    ODR_80_HZ = 0x0C                            # 80 Hz
    ODR_70_HZ = 0x0D                            # 70 Hz
    ODR_60_HZ = 0x0E                            # 60 Hz
    ODR_50_HZ = 0x0F                            # 50 Hz
    ODR_45_HZ = 0x10                            # 45 Hz
    ODR_40_HZ = 0x11                            # 40 Hz
    ODR_35_HZ = 0x12                            # 35 Hz
    ODR_30_HZ = 0x13                            # 30 Hz
    ODR_25_HZ = 0x14                            # 25 Hz
    ODR_20_HZ = 0x15                            # 20 Hz
    ODR_15_HZ = 0x16                            # 15 Hz
    ODR_10_HZ = 0x17                            # 10 Hz
    ODR_05_HZ = 0x18                            # 5 Hz
    ODR_04_HZ = 0x19                            # 4 Hz
    ODR_03_HZ = 0x1A                            # 3 Hz
    ODR_02_HZ = 0x1B                            # 2 Hz
    ODR_01_HZ = 0x1C                            # 1 Hz
    ODR_0_5_HZ = 0x1D                           # 0.5 Hz
    ODR_0_250_HZ = 0x1E                         # 0.250 Hz
    ODR_0_125_HZ = 0x1F                         # 0.125 Hz

    # Oversampling options
    OVERSAMPLING_1X = 0x00                      # 1x oversampling
    OVERSAMPLING_2X = 0x01                      # 2x oversampling
    OVERSAMPLING_4X = 0x02                      # 4x oversampling
    OVERSAMPLING_8X = 0x03                      # 8x oversampling
    OVERSAMPLING_16X = 0x04                     # 16x oversampling
    OVERSAMPLING_32X = 0x05                     # 32x oversampling
    OVERSAMPLING_64X = 0x06                     # 64x oversampling
    OVERSAMPLING_128X = 0x07                    # 128x oversampling

    # IIR filter coefficient options
    IIR_FILTER_BYPASS = 0x00                    # Bypass filter
    IIR_FILTER_COEFF_1 = 0x01                   # 1st order filter
    IIR_FILTER_COEFF_3 = 0x02                   # 3rd order filter
    IIR_FILTER_COEFF_7 = 0x03                   # 7th order filter
    IIR_FILTER_COEFF_15 = 0x04                  # 15th order filter
    IIR_FILTER_COEFF_31 = 0x05                  # 31st order filter
    IIR_FILTER_COEFF_63 = 0x06                  # 63rd order filter
    IIR_FILTER_COEFF_127 = 0x07                 # 127th order filter

    # Out-of-range count limit options
    OOR_COUNT_LIMIT_1 = 0x00                    # 1 count limit
    OOR_COUNT_LIMIT_3 = 0x01                    # 3 count limit
    OOR_COUNT_LIMIT_7 = 0x02                    # 7 count limit
    OOR_COUNT_LIMIT_15 = 0x03                   # 15 count limit

    # Power mode options
    SLEEP_MODE = 0x00                           # Standby mode
    NORMAL_MODE = 0x01                          # Normal mode
    SINGLE_SHOT_MODE = 0x02                     # Forced mode > only perform once
    CONTINOUS_MODE = 0x03                       # Continuous mode
    DEEP_SLEEP_MODE = 0x04                      # Deep standby mode

    # FIFO mode options
    FIFO_DISABLED = 0x00                        # FIFO disabled
    FIFO_TEMPERATURE_DATA = 0x01                # FIFO temperature data only enabled
    FIFO_PRESSURE_DATA = 0x02                   # FIFO pressure data only enabled
    FIFO_PRESS_TEMP_DATA = 0x03                 # FIFO pressure and temperature data enabled

    # FIFO downsampling options
    FIFO_NO_DOWNSAMPLING = 0x00                 # No downsampling
    FIFO_DOWNSAMPLING_2X = 0x01                 # 2x downsampling
    FIFO_DOWNSAMPLING_4X = 0x02                 # 4x downsampling
    FIFO_DOWNSAMPLING_8X = 0x03                 # 8x downsampling
    FIFO_DOWNSAMPLING_16X = 0x04                # 16x downsampling
    FIFO_DOWNSAMPLING_32X = 0x05                # 32x downsampling
    FIFO_DOWNSAMPLING_64X = 0x06                # 64x downsampling
    FIFO_DOWNSAMPLING_128X = 0x07               # 128x downsampling

    # FIFO operation mode options
    FIFO_STREAM_TO_FIFO_MODE = 0x00             # Stream data continuously
    FIFO_STOP_ON_FULL_MODE = 0x01               # Stop when FIFO full

    # Interrupt status options
    INT_STATUS_DRDY = 0x01                      # Data ready
    INT_STATUS_FIFO_FULL = 0x02                 # FIFO full
    INT_STATUS_FIFO_THRES = 0x04                # FIFO threshold
    INT_STATUS_PRESSURE_OOR = 0x08              # Pressure out of range
    INT_STATUS_POR_SOFTRESET_COMPLETE = 0x10    # Power on reset/soft reset complete

    # Interrupt source options
    INT_DATA_DRDY = 0x01                        # Data ready interrupt
    INT_FIFO_FULL = 0x02                        # FIFO full interrupt
    INT_FIFO_THRES = 0x04                       # FIFO threshold interrupt
    INT_PRESSURE_OOR = 0x08                     # Pressure out-of-range interrupt

    # Interrupt mode options
    INT_MODE_PULSED = 0x00                      # Pulsed mode
    INT_MODE_LATCHED = 0x01                     # Latched mode

    # Interrupt polarity options
    INT_POL_ACTIVE_LOW = 0x00                   # Active low
    INT_POL_ACTIVE_HIGH = 0x01                  # Active high

    # Interrupt output driver options
    INT_OD_PUSH_PULL = 0x00                     # Push-pull output
    INT_OD_OPEN_DRAIN = 0x01                    # Open-drain output

    # Interrupt enable options
    INT_DISABLE = 0x00                          # Interrupt disable
    INT_ENABLE = 0x01                           # Interrupt enable

    # Baud rate options
    BAUD_2400  =  0x0001                        # 2400 baud rate
    BAUD_4800  =  0x0002                        # 4800 baud rate
    BAUD_9600  =  0x0003                        # 9600 baud rate
    BAUD_14400  =  0x0004                       # 14400 baud rate
    BAUD_19200  =  0x0005                       # 19200 baud rate
    BAUD_38400  =  0x0006                       # 38400 baud rate
    BAUD_57600  =  0x0007                       # 57600 baud rate
    BAUD_115200  =  0x0008                      # 115200 baud rate

    class sFIFOData_t(Structure):
        _fields_ = [
            ("temperature", c_float * 32),
            ("pressure", c_float * 32),
            ("len", c_uint8)
        ]

        def clear(self):
            self.len = 0
            for i in range(32):
                self.pressure[i] = 0.0
            for i in range(32):
                self.temperature[i] = 0.0

    def __init__(self):
        pass

    def begin(self):
        '''!
          @fn begin
          @brief Initializes the sensor hardware interface
          @return true if initialization succeeds, false on failure
        '''
        self.reset()
        chip_data = self._read_input_reg(self.REG_I_CHIP_ID, 1)
        self._calibrated = False
        self._sealevelAltitude = 0.0
        if chip_data[0] not in (self.BMP581_CHIP_ID, self.BMP585_CHIP_ID):
            return False
        self._enable_pressure(self.ENABLE)
        return True

    def set_odr(self, odr):
        '''!
          @fn set_odr
          @brief Configures sensor output data rate (ODR)
          @param odr Output data rate selection
          @n Available rates:
          @n - ODR_240_HZ:    240 Hz
          @n - ODR_218_5_HZ:  218.5 Hz
          @n - ODR_199_1_HZ:  199.1 Hz
          @n - ODR_179_2_HZ:  179.2 Hz
          @n - ODR_160_HZ:    160 Hz
          @n - ODR_149_3_HZ:  149.3 Hz
          @n - ODR_140_HZ:    140 Hz
          @n - ODR_129_8_HZ:  129.8 Hz
          @n - ODR_120_HZ:    120 Hz
          @n - ODR_110_1_HZ:  110.1 Hz
          @n - ODR_100_2_HZ:  100.2 Hz
          @n - ODR_89_6_HZ:   89.6 Hz
          @n - ODR_80_HZ:     80 Hz
          @n - ODR_70_HZ:     70 Hz
          @n - ODR_60_HZ:     60 Hz
          @n - ODR_50_HZ:     50 Hz
          @n - ODR_45_HZ:     45 Hz
          @n - ODR_40_HZ:     40 Hz
          @n - ODR_35_HZ:     35 Hz
          @n - ODR_30_HZ:     30 Hz
          @n - ODR_25_HZ:     25 Hz
          @n - ODR_20_HZ:     20 Hz
          @n - ODR_15_HZ:     15 Hz
          @n - ODR_10_HZ:     10 Hz
          @n - ODR_05_HZ:     5 Hz
          @n - ODR_04_HZ:     4 Hz
          @n - ODR_03_HZ:     3 Hz
          @n - ODR_02_HZ:     2 Hz
          @n - ODR_01_HZ:     1 Hz
          @n - ODR_0_5_HZ:    0.5 Hz
          @n - ODR_0_250_HZ:  0.250 Hz
          @n - ODR_0_125_HZ:  0.125 Hz
          @return true if configuration succeeds, false on failure
        '''
        if odr not in range(self.ODR_240_HZ, self.ODR_0_125_HZ + 1):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.ODR_POS, self.ODR_WIDTH, odr)
        self._write_holding_reg(self.REG_H_ODR_CONFIG, data)
        return True

    def set_osr(self, osr_temp, osr_press):
        '''!
          @fn set_osr
          @brief Sets oversampling ratios for temperature and pressure
          @param osr_temp Temperature oversampling
          @param osr_press Pressure oversampling
          @n Supported values:
          @n - OVERSAMPLING_1X:   1x oversampling
          @n - OVERSAMPLING_2X:   2x oversampling
          @n - OVERSAMPLING_4X:   4x oversampling
          @n - OVERSAMPLING_8X:   8x oversampling
          @n - OVERSAMPLING_16X:  16x oversampling
          @n - OVERSAMPLING_32X:  32x oversampling
          @n - OVERSAMPLING_64X:  64x oversampling
          @n - OVERSAMPLING_128X: 128x oversampling
          @return true if configuration succeeds, false on failure
        '''
        if osr_temp not in range(self.OVERSAMPLING_1X, self.OVERSAMPLING_128X + 1):
            return False
        if osr_press not in range(self.OVERSAMPLING_1X, self.OVERSAMPLING_128X + 1):
            return False
        data = self._read_holding_reg(self.REG_H_OSR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.OSR_T_POS, self.OSR_T_WIDTH, osr_temp)
        data[0] = self._REG_SET_BITS(
            data[0], self.OSR_P_POS, self.OSR_P_WIDTH, osr_press)
        self._write_holding_reg(self.REG_H_OSR_CONFIG, data)
        return True

    def set_measure_mode(self, mode):
        '''!
          @fn setMeasureMode
          @brief  set the measurement mode of the sensor
          @param  mode: measurement mode
          @n      SLEEP_MODE = 0x00.        #// standby mode
          @n      NORMAL_MODE = 0x01.        #// normal mode
          @n      SINGLE_SHOT_MODE = 0x02.         #// forced mode > only perform once
          @n      CONTINOUS_MODE = 0x03.      #// continuous mode
          @n      DEEP_SLEEP_MODE = 0x04.   #// deep standby mode
          @return True if the setting is successful, False otherwise.
        '''
        if mode not in (self.SLEEP_MODE, self.NORMAL_MODE, self.SINGLE_SHOT_MODE, self.CONTINOUS_MODE, self.DEEP_SLEEP_MODE):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        currMode = self._REG_GET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH)
        if currMode != self.SLEEP_MODE:
            self._set_power_mode(self.SLEEP_MODE)
            time.sleep(0.0025)
        if mode == self.DEEP_SLEEP_MODE:
            self._set_deep_standby_mode(self.DEEP_ENABLE)
        elif mode in (self.CONTINOUS_MODE, self.SINGLE_SHOT_MODE, self.NORMAL_MODE):
            self._set_power_mode(mode)
        return True

    def reset(self):
        '''!
          @fn reset
          @brief  reset the sensor
          @return True if the reset is successful, False otherwise
        '''
        self._write_holding_reg(self.REG_H_CMD, self.SOFT_RESET_CMD)
        time.sleep(0.002)
        self._read_input_reg(self.REG_I_CHIP_ID, 1)

        intStatus = self.get_int_status()
        if intStatus & self.INT_STATUS_POR_SOFTRESET_COMPLETE:
            return self._enable_pressure(self.ENABLE)
        return False

    def read_temperature(self):
        '''!
          @fn read_temperature
          @brief  read the temperature of the sensor
          @return temperature in Celsius
        '''
        data = self._read_input_reg(self.REG_I_TEMP_DATA_XLSB, 3)

        tmpData = self.convert_data(data)

        return tmpData / 65536.0

    def read_pressure(self):
        '''!
          @fn read_pressure
          @brief  read the pressure of the sensor
          @return pressure in Pascal
        '''
        data = self._read_input_reg(self.REG_I_PRESS_DATA_XLSB, 3)

        pressData = self.convert_data(data) / 64.0
        return pressData

    def read_altitude(self):
        '''!
          @fn read_altitude
          @brief  read the altitude of the sensor
          @return altitude in meters
        '''
        data = self._read_input_reg(self.REG_I_TEMP_DATA_XLSB, 6)

        tmpData = self.convert_data(data)

        pressData = self.convert_data(data[3:6])

        temperature = tmpData / 65536.0
        pressure = pressData / 64.0
        if self._calibrated:
            seaLevelPressPa = (pressure / (1.0 - (self._sealevelAltitude / 44307.7)) ** 5.255302)
            pressure = pressure - seaLevelPressPa + self.STANDARD_SEA_LEVEL_PRESSURE_PA
        return self._calculate_altitude(temperature, pressure)

    def config_iir(self, iir_t, iir_p):
        '''!
          @fn config_iir
          @brief Configures IIR filter coefficients
          @param iir_t Temperature IIR filter
          @param iir_p Pressure IIR filter
          @n Available coefficients:
          @n - IIR_FILTER_BYPASS:   Bypass filter
          @n - IIR_FILTER_COEFF_1:  1st order filter
          @n - IIR_FILTER_COEFF_3:  3rd order filter
          @n - IIR_FILTER_COEFF_7:  7th order filter
          @n - IIR_FILTER_COEFF_15: 15th order filter
          @n - IIR_FILTER_COEFF_31: 31st order filter
          @n - IIR_FILTER_COEFF_63: 63rd order filter
          @n - IIR_FILTER_COEFF_127:127th order filter
          @return True if configuration is successful, False otherwise
        '''
        if iir_t not in range(self.IIR_FILTER_BYPASS, self.IIR_FILTER_COEFF_127 + 1):
            return False
        if iir_p not in range(self.IIR_FILTER_BYPASS, self.IIR_FILTER_COEFF_127 + 1):
            return False

        currMode = self._get_power_mode()
        self.set_measure_mode(self.SLEEP_MODE)
        data = self._read_holding_reg(self.REG_H_DSP_CONFIG, 2)
        data[0] = self._REG_SET_BITS(
            data[0], self.SHDW_SEL_IIR_T_POS, self.SHDW_SEL_IIR_T_WIDTH, self.ENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.SHDW_SEL_IIR_P_POS, self.SHDW_SEL_IIR_P_WIDTH, self.ENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.IIR_FLUSH_FORCED_EN_POS, self.IIR_FLUSH_FORCED_EN_WIDTH, self.ENABLE)

        data[1] = self._REG_SET_BITS(
            data[1], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH, iir_t)
        data[1] = self._REG_SET_BITS(
            data[1], self.SET_IIR_T_POS, self.SET_IIR_P_WIDTH, iir_p)

        self._write_holding_reg(self.REG_H_DSP_CONFIG, data)
        if currMode not in (self.DEEP_SLEEP_MODE, self.SLEEP_MODE):
            return self.set_measure_mode(currMode)
        return True

    def config_fifo(self, frame_sel=FIFO_PRESS_TEMP_DATA, dec_sel=FIFO_NO_DOWNSAMPLING, mode=FIFO_STREAM_TO_FIFO_MODE, threshold=0x00):
        '''!
          @fn config_fifo
          @brief Configures FIFO operation parameters
          @param frame_sel Data frame type
          @n Available types:
          @n - FIFO_DISABLED:    FIFO disabled
          @n - FIFO_TEMPERATURE_DATA: Temperature data only
          @n - FIFO_PRESSURE_DATA:    Pressure data only
          @n - FIFO_PRESS_TEMP_DATA:  Pressure and temperature data

          @param dec_sel Downsampling ratio
          @n Available ratios:
          @n - FIFO_NO_DOWNSAMPLING: No downsampling
          @n - FIFO_DOWNSAMPLING_2X:  2x downsampling
          @n - FIFO_DOWNSAMPLING_4X:  4x downsampling
          @n - FIFO_DOWNSAMPLING_8X:  8x downsampling
          @n - FIFO_DOWNSAMPLING_16X: 16x downsampling
          @n - FIFO_DOWNSAMPLING_32X: 32x downsampling
          @n - FIFO_DOWNSAMPLING_64X: 64x downsampling
          @n - FIFO_DOWNSAMPLING_128X:128x downsampling

          @param mode FIFO operation mode
          @n Available modes:
          @n - FIFO_STREAM_TO_FIFO_MODE: Stream data continuously
          @n - FIFO_STOP_ON_FULL_MODE:   Stop when FIFO full

          @param threshold FIFO trigger threshold (0=disable, 1-31=frames)]
          @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most
          significant bit is ignored.
          @n - 0x1F: 31 frames. This is the maximum setting in P- or T-mode.
          @return True if configuration is successful, False otherwise.
        '''
        if frame_sel not in range(self.FIFO_DISABLED, self.FIFO_PRESS_TEMP_DATA + 1):
            return False
        if dec_sel not in range(self.FIFO_NO_DOWNSAMPLING, self.FIFO_DOWNSAMPLING_128X + 1):
            return False
        if mode not in (self.FIFO_STREAM_TO_FIFO_MODE, self.FIFO_STOP_ON_FULL_MODE):
            return False
        if frame_sel == self.FIFO_TEMPERATURE_DATA or frame_sel == self.FIFO_PRESSURE_DATA:
            if threshold > 0x1F:
                return False
        elif frame_sel == self.FIFO_PRESS_TEMP_DATA:
            if threshold > 0x0F:
                return False
        currMode = self._get_power_mode()
        self.set_measure_mode(self.SLEEP_MODE)
        data = self._read_holding_reg(self.REG_H_DSP_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_SEL_IIR_T_POS, self.FIFO_SEL_IIR_T_WIDTH, self.ENABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_SEL_IIR_P_POS, self.FIFO_SEL_IIR_P_WIDTH, self.ENABLE)
        self._write_holding_reg(self.REG_H_DSP_CONFIG, data)

        data = self._read_holding_reg(self.REG_H_FIFO_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_MODE_POS, self.FIFO_MODE_WIDTH, mode)
        data[0] = self._set_fifo_threshold(data[0], frame_sel, threshold)
        self._write_holding_reg(self.REG_H_FIFO_CONFIG, data)

        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH, frame_sel)
        data[0] = self._REG_SET_BITS(
            data[0], self.FIFO_DEC_SEL_POS, self.FIFO_DEC_SEL_WIDTH, dec_sel)
        self._write_holding_reg(self.REG_H_FIFO_SEL, data)

        if currMode not in (self.DEEP_SLEEP_MODE, self.SLEEP_MODE):
            return self.set_measure_mode(currMode)
        return True

    def get_fifo_count(self):
        '''!
          @fn get_fifo_count
          @brief  Get the number of frames in the FIFO.
          @return Number of frames in the FIFO.
        '''
        data = self._read_input_reg(self.REG_I_FIFO_COUNT, 1)
        count = self._REG_GET_BITS(
            data[0], self.FIFO_COUNT_POS, self.FIFO_COUNT_WIDTH)
        return count

    def get_fifo_data(self):
        '''!
          @fn getFIFOData
          @brief Reads all data from FIFO
          @return sFIFOData_t Struct containing pressure and temperature data
          @n - len: Number of stored data frames (0-31)
          @n - pressure: Array of pressure values
          @n - temperature: Array of temperature values
        '''
        fifo_data = self.sFIFOData_t()
        fifo_data.clear()
        fifo_count = self.get_fifo_count()
        fifo_data.len = fifo_count
        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        fifo_frame_sel = self._REG_GET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH)
        if fifo_frame_sel == self.FIFO_PRESSURE_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 3)
                pressure = self.convert_data(data) / 64.0
                fifo_data.pressure[i] = pressure
        elif fifo_frame_sel == self.FIFO_TEMPERATURE_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 3)
                fifo_data.temperature[i] = self.convert_data(data) / 65536.0
        elif fifo_frame_sel == self.FIFO_PRESS_TEMP_DATA:
            for i in range(fifo_count):
                data = self._read_input_reg(self.REG_I_FIFO_DATA, 6)
                tmpData = self.convert_data(data)
                pressData = self.convert_data(data[3:6])
                fifo_data.temperature[i] = tmpData / 65536.0
                pressure = pressData / 64.0
                fifo_data.pressure[i] = pressure
        return fifo_data

    def config_interrupt(self, int_mode, int_pol, int_od):
        '''!
          @fn config_interrupt
          @brief Configures interrupt behavior
          @param int_mode Trigger mode
          @n Available modes:
          @n - INT_MODE_PULSED: Pulsed mode
          @n - INT_MODE_LATCHED: Latched mode

          @param int_pol Signal polarity
          @n Available polarities:
          @n - INT_POL_ACTIVE_LOW: Active low
          @n - INT_POL_ACTIVE_HIGH: Active high

          @param int_od Output driver type
          @n Available types:
          @n - INT_OD_PUSH_PULL: Push-pull output
          @n - INT_OD_OPEN_DRAIN: Open-drain output
          @return True if configuration is successful, False otherwise.
        '''
        if int_mode not in (self.INT_MODE_PULSED, self.INT_MODE_LATCHED):
            return False
        if int_pol not in (self.INT_POL_ACTIVE_LOW, self.INT_POL_ACTIVE_HIGH):
            return False
        if int_od not in (self.INT_OD_PUSH_PULL, self.INT_OD_OPEN_DRAIN):
            return False
        '''
          Any change between latched/pulsed mode has to be applied while interrupt is disabled
          Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00)
        '''
        self._write_holding_reg(self.REG_H_INT_SOURCE, [0x00])
        data = self._read_holding_reg(self.REG_H_INT_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_MODE_POS, self.INT_MODE_WIDTH, int_mode)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_POL_POS, self.INT_POL_WIDTH, int_pol)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_OD_POS, self.INT_OD_WIDTH, int_od)
        data[0] = self._REG_SET_BITS(
            data[0], self.INT_EN_POS, self.INT_EN_WIDTH, self.INT_ENABLE)
        self._write_holding_reg(self.REG_H_INT_CONFIG, data)
        return True

    def set_int_source(self, source):
        '''!
          @fn set_int_source
          @brief Enables specific interrupt sources
          @param source Bitmask of triggers
          @n Available sources:
          @n - INT_DATA_DRDY:    Data ready interrupt
          @n - INT_FIFO_FULL:    FIFO full interrupt
          @n - INT_FIFO_THRES:   FIFO threshold interrupt
          @n - INT_PRESSURE_OOR: Pressure out-of-range interrupt
          @return True if configuration is successful, False otherwise.
        '''
        VALID_INT_MASK = self.INT_DATA_DRDY | self.INT_FIFO_FULL | self.INT_FIFO_THRES | self.INT_PRESSURE_OOR
        if source & ~VALID_INT_MASK:
            return False
        data = source & VALID_INT_MASK
        self._write_holding_reg(self.REG_H_INT_SOURCE, data)
        return True

    def get_int_status(self):
        '''!
            @brief  Get the interrupt status of the sensor.
            @return Interrupt status.
            @n      INT_STATUS_DRDY = 0x01,                   # // data ready
            @n      INT_STATUS_FIFO_FULL = 0x02,              # // FIFO full
            @n      INT_STATUS_FIFO_THRES = 0x04,             # // FIFO threshold
            @n      INT_STATUS_PRESSURE_OOR = 0x08,           # // pressure out of range
            # // power on reset/soft reset complete
            @n      INT_STATUS_POR_SOFTRESET_COMPLETE = 0x10,
        '''
        data = self._read_input_reg(self.REG_I_INT_STATUS, 1)
        return data[0]

    def set_oor_press(self, oor, range_val, cnt_lim):
        '''!
            @brief  Set the out of range_val pressure of the sensor.
            @param  oor: Out of range_val pressure.
            @n      0x00000 - 0x1FFFF: Out of range_val pressure
            @param  range_val: Out of range pressure range_val.
            @n      0x00 - 0xFF: Out of range pressure range_val  (oor - range_val, oor + range_val)
            @param  cnt_lim: Out of range pressure count limit.
            @n      OOR_COUNT_LIMIT_1  = 0x00
            @n      OOR_COUNT_LIMIT_3  = 0x01
            @n      OOR_COUNT_LIMIT_7  = 0x02
            @n      OOR_COUNT_LIMIT_15 = 0x03
            @return True if configuration is successful, False otherwise.
        '''
        if oor not in range(0x00000, 0x1FFFF + 1):
            return False
        if range_val not in range(0x00, 0xFF + 1):
            return False
        if cnt_lim not in (self.OOR_COUNT_LIMIT_1, self.OOR_COUNT_LIMIT_3, self.OOR_COUNT_LIMIT_7, self.OOR_COUNT_LIMIT_15):
            return False
        currMode = self._get_power_mode()
        self.set_measure_mode(self.SLEEP_MODE)

        data = self._read_holding_reg(self.REG_H_OOR_THR_P_LSB, 4)
        data[0] = oor & 0xFF
        data[1] = (oor >> 8) & 0xFF
        data[3] = self._REG_SET_BITS(data[3], self.OOR_THR_P_16_POS,
                           self.OOR_THR_P_16_WIDTH, (oor >> 16) & 0x01)

        data[2] = range_val
        data[3] = self._REG_SET_BITS(data[3], self.CNT_LIM_POS,
                           self.CNT_LIM_WIDTH, cnt_lim)
        self._write_holding_reg(self.REG_H_OOR_THR_P_LSB, data)
        if currMode not in (self.DEEP_SLEEP_MODE, self.SLEEP_MODE):
            return self.set_measure_mode(currMode)
        return True

    def calibrated_absolute_difference(self, altitude):
        '''!
            @fn calibratedAbsoluteDifference
            @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent pressure and altitude data
            @param altitude current altitude
            @return boolean, indicates whether the reference value is set successfully
            @retval True indicates the reference value is set successfully
            @retval False indicates fail to set the reference value
        '''
        ret = False
        if altitude > 0:
            self._calibrated = True
            self._sealevelAltitude = altitude
            ret = True
        return ret

    def _enable_pressure(self, enable):
        '''!
            @brief  enable or disable pressure measurement
            @param  enable: enable or disable.
            @n      eENABLE = 0x01
            @n      eDISABLE = 0x00
            @return True if the setting is successful, False otherwise
        '''
        if enable not in (self.ENABLE, self.DISABLE):
            return False
        data = self._read_holding_reg(self.REG_H_OSR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.PRESS_EN_POS, self.PRESS_EN_WIDTH, enable)
        self._write_holding_reg(self.REG_H_OSR_CONFIG, data)
        return True

    def _set_deep_standby_mode(self, deepMode):
        '''!
            @brief  Set the deep standby mode of the sensor.
            @param  deepMode: Deep standby mode.
            @n      DEEP_DISABLE = 0x00,
            @n      DEEP_ENABLE = 0x01,
            @return True if configuration is successful, False otherwise.
        '''
        if deepMode == self.DEEP_ENABLE:
            data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH, self.DEEP_ENABLE)
            data[0] = self._REG_SET_BITS(
                data[0], self.ODR_POS, self.ODR_WIDTH, self.ODR_01_HZ)
            self._write_holding_reg(self.REG_H_ODR_CONFIG, data)

            data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH, self.FIFO_DISABLED)
            self._write_holding_reg(self.REG_H_FIFO_SEL, data)

            data = self._read_holding_reg(self.REG_H_DSP_IIR, 1)
            data[0] = self._REG_SET_BITS(
                data[0], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH, self.IIR_FILTER_BYPASS)
            data[0] = self._REG_SET_BITS(
                data[0], self.SET_IIR_P_POS, self.SET_IIR_P_WIDTH, self.IIR_FILTER_BYPASS)
            self._write_holding_reg(self.REG_H_DSP_IIR, data)
        else:
            currMode = self._get_power_mode()
            if currMode == self.DEEP_SLEEP_MODE:
                return self.set_measure_mode(self.SLEEP_MODE)
        return True

    def _set_power_mode(self, mode):
        '''!
            @brief  Set the power mode of the sensor.
            @param  mode: Power mode.
            @n      SLEEP_MODE = 0x00,
            @n      NORMAL_MODE = 0x01,
            @n      SINGLE_SHOT_MODE = 0x02,
            @n      CONTINOUS_MODE = 0x03,
            @n      DEEP_SLEEP_MODE = 0x04,
            @return True if configuration is successful, False otherwise.
        '''
        if mode not in (self.SLEEP_MODE, self.NORMAL_MODE, self.SINGLE_SHOT_MODE, self.CONTINOUS_MODE, self.DEEP_SLEEP_MODE):
            return False
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        data[0] = self._REG_SET_BITS(
            data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH, self.DEEP_DISABLE)
        data[0] = self._REG_SET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH, mode)
        self._write_holding_reg(self.REG_H_ODR_CONFIG, data)
        return True

    def _get_power_mode(self):
        '''!
            @brief  Get the power mode of the sensor.
            @return Power mode.
            @n      SLEEP_MODE = 0x00,
            @n      NORMAL_MODE = 0x01,
            @n      SINGLE_SHOT_MODE = 0x02,
            @n      CONTINOUS_MODE = 0x03,
            @n      DEEP_SLEEP_MODE = 0x04,
        '''
        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        currMode = self._REG_GET_BITS(
            data[0], self.PWR_MODE_POS, self.PWR_MODE_WIDTH)
        deep_dis = self._REG_GET_BITS(
            data[0], self.DEEP_DIS_POS, self.DEEP_DIS_WIDTH)
        if currMode == self.SLEEP_MODE and deep_dis == self.DEEP_ENABLE:
            return self._verify_deep_standby_mode()
        return currMode

    def _verify_deep_standby_mode(self):
        '''!
            @brief  Verify the deep standby mode of the sensor.
            @return True if the deep standby mode is verified, False otherwise.
        '''
        mode = self.SLEEP_MODE
        data = self._read_holding_reg(self.REG_H_FIFO_SEL, 1)
        fifo_frame_sel = self._REG_GET_BITS(
            data[0], self.FIFO_FRAME_SEL_POS, self.FIFO_FRAME_SEL_WIDTH)

        data = self._read_holding_reg(self.REG_H_ODR_CONFIG, 1)
        odr = self._REG_GET_BITS(data[0], self.ODR_POS, self.ODR_WIDTH)

        data = self._read_holding_reg(self.REG_H_DSP_IIR, 1)
        iir_t = self._REG_GET_BITS(
            data[0], self.SET_IIR_T_POS, self.SET_IIR_T_WIDTH)
        iir_p = self._REG_GET_BITS(
            data[0], self.SET_IIR_P_POS, self.SET_IIR_P_WIDTH)

        if odr > self.ODR_05_HZ and fifo_frame_sel == self.FIFO_DISABLED and iir_t == self.IIR_FILTER_BYPASS and iir_p == self.IIR_FILTER_BYPASS:
            mode = self.SLEEP_MODE
        return mode

    def _set_fifo_threshold(self, data, frame_sel, threshold):
        '''!
            @brief  Set the FIFO threshold of the sensor.
            @param  data: Data to be set.
            @param  frame_sel: Frame selection.
            @param  threshold: Threshold.
            @return Data to be set.
        '''
        if frame_sel == self.FIFO_TEMPERATURE_DATA or frame_sel == self.FIFO_PRESSURE_DATA:
            if threshold <= 0x1F:
                return self._REG_SET_BITS(data, self.FIFO_THRESHOLD_POS, self.FIFO_THRESHOLD_WIDTH, threshold)
        elif frame_sel == self.FIFO_PRESS_TEMP_DATA:
            if threshold <= 0x0F:
                return self._REG_SET_BITS(data, self.FIFO_THRESHOLD_POS, self.FIFO_THRESHOLD_WIDTH, threshold)
        return 0

    def _calculate_altitude(self, temperature_c, pressure_pa):
        '''!
            @brief  Calculate the altitude of the sensor.
            @param  temperature_c: Temperature in Celsius.
            @param  pressure_pa: Pressure in Pascal.
            @return Altitude in meters.
        '''
        return (1.0 - (pressure_pa / 101325) ** 0.190284) * 44307.7

    def _REG_BIT_MASK(self, pos, width):
        return ((1 << width) - 1) << pos

    def _REG_SET_BITS(self, reg, pos, width, val):
        cleared_reg = reg & ~self._REG_BIT_MASK(pos, width)
        return cleared_reg | ((val & ((1 << width) - 1)) << pos)

    def _REG_GET_BITS(self, reg, pos, width):
        return (reg >> pos) & ((1 << width) - 1)

    def convert_data(self, data):
        '''!
            @brief  Convert data to int.
            @param  data: Data to be converted.
            @return Converted data.
        '''
        byte0 = data[0] & 0xFF
        byte1 = data[1] & 0xFF
        byte2 = data[2] & 0xFF

        if byte2 & 0x80:
            byte2_signed = byte2 - 0x100
        else:
            byte2_signed = byte2

        data = (byte2_signed << 16) | (byte1 << 8) | byte0
        return data

    def _write_holding_reg(self, reg, data):
        pass

    def _read_input_reg(self, reg, size):
        pass

    def _read_holding_reg(self, reg, size):
        pass


class DFRobot_BMP58X_I2C(DFRobot_BMP58X):
    def __init__(self, bus, addr):
        self.__addr = addr
        self.__i2cbus = smbus.SMBus(bus)
        super(DFRobot_BMP58X_I2C, self).__init__()

    def _write_holding_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        ret = self.__i2cbus.write_i2c_block_data(self.__addr, reg, data)
        time.sleep(0.002)
        return ret

    def _read_input_reg(self, reg, size):
        return self.__i2cbus.read_i2c_block_data(self.__addr, reg, size)

    def _read_holding_reg(self, reg, size):
        return self.__i2cbus.read_i2c_block_data(self.__addr, reg, size)


class DFRobot_BMP58X_SPI(DFRobot_BMP58X):
    def __init__(self, cs=8, bus=0, dev=0, speed=8000000):
        self.__cs = cs
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.__cs, GPIO.OUT, initial=1)
        GPIO.output(self.__cs, GPIO.LOW)
        self.__spi = spidev.SpiDev()
        self.__spi.open(bus, dev)
        self.__spi.no_cs = True
        self.__spi.max_speed_hz = speed
        super(DFRobot_BMP58X_SPI, self).__init__()

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        if isinstance(data, int):
            data = [data]
            # logger.info(data)
        reg_addr = [reg & 0x7f]
        GPIO.output(self.__cs, GPIO.LOW)
        self.__spi.xfer(reg_addr)
        self.__spi.xfer(data)
        GPIO.output(self.__cs, GPIO.HIGH)

    def _read_reg(self, reg, size):
        '''!
          @brief read the data from the register
          @param reg register address
          @param size read data length
          @return read data list
        '''
        reg_addr = [reg | 0x80]
        GPIO.output(self.__cs, GPIO.LOW)
        # logger.info(reg_addr)
        self.__spi.xfer(reg_addr)
        time.sleep(0.01)
        # self.__spi.readbytes(1)
        rslt = self.__spi.readbytes(size)
        GPIO.output(self.__cs, GPIO.HIGH)
        return rslt

    def _write_holding_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        idx = 0
        for val in data:
            self._write_reg(reg + idx, val)
            idx += 1

    def _read_input_reg(self, reg, size):
        return self._read_reg(reg, size)

    def _read_holding_reg(self, reg, size):
        return self._read_reg(reg, size)


class DFRobot_BMP58X_UART(DFRobot_BMP58X, DFRobot_RTU):
    def __init__(self, baud, addr):
        self.__baud = baud
        self.__addr = addr
        DFRobot_BMP58X.__init__(self)
        DFRobot_RTU.__init__(self, baud, 8, 'N', 1)
    def set_baud(self, baud):
        '''!
          @fn setBaud
          @brief Set the UART communication baud rate.
          @details Configures the serial communication speed using the specified baud rate enum.
                   This function initializes the necessary hardware registers to achieve the desired
                   data transfer rate.
          @param baud An eBaud enum value specifying the desired baud rate.
                      Defaults to e9600 if not explicitly set.
          @note Actual hardware configuration may vary depending on the microcontroller model.
                The function assumes a standard clock frequency; adjust clock settings
                if using non-default system clock configurations.
          @warning Changing the baud rate during communication may cause data loss
                   or communication errors if both devices are not synchronized.
          @see eBaud for available baud rate options:
               - BAUD_2400: 2400 bits per second
               - BAUD_4800: 4800 bits per second
               - BAUD_9600: 9600 bits per second (default)
               - BAUD_14400: 14400 bits per second
               - BAUD_19200: 19200 bits per second
               - BAUD_38400: 38400 bits per second
               - BAUD_57600: 57600 bits per second
               - BAUD_115200: 115200 bits per second
        '''
        baud = baud & 0xFF
        self.write_holding_register(self.__addr, self.REG_H_MODBUS_BAUD, baud)

    def _write_holding_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        tmp_data = []
        for i in data:
            tmp_data.append((i >> 8) & 0xFF)
            tmp_data.append(i & 0xFF)
        ret = self.write_holding_registers(self.__addr, reg + self.OFFSET_REG, tmp_data)
        return ret

    def _read_input_reg(self, reg, size):
        try:
            data = self.read_input_registers(self.__addr, reg + self.OFFSET_REG, size)
            ret_data = []
            for i in range(size):
                ret_data.append((data[i*2+1] << 8) | data[i*2+2])
            return ret_data
        except Exception as e:
            return [0]

    def _read_holding_reg(self, reg, size):
        try:
            data = self.read_holding_registers(self.__addr, reg + self.OFFSET_REG, size)
            ret_data = []
            for i in range(size):
                ret_data.append((data[i*2+1] << 8) | data[i*2+2])
            return ret_data
        except Exception as e:
            return [0]
