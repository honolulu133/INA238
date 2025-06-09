/**
 * MIT License

 * Copyright (c) 2025 honolulu133

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INA238_H
#define INA238_H

#include <Arduino.h>
#include <Wire.h>

class INA238
{
    // Public enumeration for INA238 configuration options
public:
    /**
     * @brief Enumeration for ADC range options.
     * This enum defines the range of the ADC used in the INA238.
     * The INA238 supports two ADC ranges:
     * - RANGE_163_84_MV: This range corresponds to a full-scale input of +-163.84 mV.
     * - RANGE_40_96_MV: This range corresponds to a full-scale input of +-40.96 mV.
     */
    enum class ADCRange : uint8_t {
        RANGE_163_84_MV = 0, // +-163.84 mV
        RANGE_40_96_MV = 1   // +-40.96 mV
    };

    /**
     * @brief Enumeration for mode selection.
     * This enum defines the operational modes of the INA238.
     */

    enum class MeasurementMode : uint8_t {
        SHUTDOWN = 0x00,                                      // 0h = Shutdown
        TRIGGERED_BUS_VOLTAGE_SINGLE_SHOT = 0x01,             // 1h = Triggered bus voltage, single shot
        TRIGGERED_SHUNT_VOLTAGE_SINGLE_SHOT = 0x02,           // 2h = Triggered shunt voltage, single shot
        TRIGGERED_SHUNT_AND_BUS_VOLTAGE_SINGLE_SHOT = 0x03,   // 3h = Triggered shunt voltage and bus voltage, single shot
        TRIGGERED_TEMPERATURE_SINGLE_SHOT = 0x04,             // 4h = Triggered temperature, single shot
        TRIGGERED_TEMPERATURE_AND_BUS_VOLTAGE_SINGLE_SHOT = 0x05, // 5h = Triggered temperature and bus voltage, single shot
        TRIGGERED_TEMPERATURE_AND_SHUNT_VOLTAGE_SINGLE_SHOT = 0x06, // 6h = Triggered temperature and shunt voltage, single shot
        TRIGGERED_ALL_SINGLE_SHOT = 0x07,                     // 7h = Triggered bus voltage, shunt voltage and temperature, single shot
        SHUTDOWN_ALT = 0x08,                                  // 8h = Shutdown (alternate code, if needed, or just map to SHUTDOWN)
        CONTINUOUS_BUS_VOLTAGE = 0x09,                        // 9h = Continuous bus voltage only
        CONTINUOUS_SHUNT_VOLTAGE = 0x0A,                      // Ah = Continuous shunt voltage only
        CONTINUOUS_SHUNT_AND_BUS_VOLTAGE = 0x0B,              // Bh = Continuous shunt and bus voltage
        CONTINUOUS_TEMPERATURE = 0x0C,                        // Ch = Continuous temperature only
        CONTINUOUS_BUS_VOLTAGE_AND_TEMPERATURE = 0x0D,        // Dh = Continuous bus voltage and temperature
        CONTINUOUS_TEMPERATURE_AND_SHUNT_VOLTAGE = 0x0E,      // Eh = Continuous temperature and shunt voltage
        CONTINUOUS_ALL = 0x0F                                 // Fh = Continuous bus voltage, shunt voltage and temperature
    };

    /**
     * @brief Enumeration for conversion time options.
     * This enum defines the conversion time for the ADC in the INA238.
     */
    enum class ConversionTime : uint8_t {
        CONVERSION_TIME_50_US = 0x00,    // 0h = 50 μs
        CONVERSION_TIME_84_US = 0x01,    // 1h = 84 μs
        CONVERSION_TIME_150_US = 0x02,   // 2h = 150 μs
        CONVERSION_TIME_280_US = 0x03,   // 3h = 280 μs
        CONVERSION_TIME_540_US = 0x04,   // 4h = 540 μs
        CONVERSION_TIME_1052_US = 0x05,  // 5h = 1052 μs
        CONVERSION_TIME_2074_US = 0x06,  // 6h = 2074 μs
        CONVERSION_TIME_4120_US = 0x07   // 7h = 4120 μs
    };

    /**
     * @brief Enumeration for averaging options.
     * This enum defines the number of samples to average in the ADC.
     * The INA238 supports two averaging options:
     * - AVERAGE_1: 1 sample, output registers are updated every conversion time.
     * - AVERAGE_x: x samples, output registers are updated after x conversion times.
     */
    enum class Averaging : uint8_t {
        AVERAGE_1 = 0x00,   // 0h = 1 sample
        AVERAGE_4 = 0x01,   // 1h = 4 samples
        AVERAGE_16 = 0x02,  // 2h = 16 samples
        AVERAGE_64 = 0x03,  // 3h = 64 samples
        AVERAGE_128 = 0x04, // 4h = 128 samples
        AVERAGE_256 = 0x05, // 5h = 256 samples
        AVERAGE_512 = 0x06, // 6h = 512 samples
        AVERAGE_1024 = 0x07 // 7h = 1024 samples
    };

    // Public constants for INA238 registers
    public:
        static const uint8_t INA238_REG_CONFIG = 0x00;
        static const uint8_t INA238_REG_ADC_CONFIG = 0x01;
        static const uint8_t INA238_REG_SHUNT_CAL = 0x02;
        static const uint8_t INA238_REG_VSHUNT = 0x04;
        static const uint8_t INA238_REG_VBUS = 0x05;
        static const uint8_t INA238_REG_DIETEMP = 0x06;
        static const uint8_t INA238_REG_CURRENT = 0x07;
        static const uint8_t INA238_REG_POWER = 0x08;
        static const uint8_t INA238_REG_DIAG_ALRT = 0x0B;
        static const uint8_t INA238_REG_SOFL = 0x0C;
        static const uint8_t INA238_REG_SUFL = 0x0D;
        static const uint8_t INA238_REG_BOFL = 0x0E;
        static const uint8_t INA238_REG_BUFL = 0x0F;
        static const uint8_t INA238_REG_TEMP_LIMIT = 0x10;
        static const uint8_t INA238_REG_PWR_LIMIT = 0x11;
        static const uint8_t INA238_REG_MANUFACTURER_ID = 0x3E;
        static const uint8_t INA238_REG_DEVICE_ID = 0x3F;

    // Config register structure
    struct ConfigRegister {
        uint16_t reset : 1; // 0: Normal operation, 1: Reset
        uint16_t reserved1 : 1; // Reserved bit
        uint16_t convDly : 8; // Conversion delay, 0 - 510 ms, step 2 ms
        uint16_t reserved2 : 1; // Reserved bit
        uint16_t adcRange : 1; // 0: +-163.84 mV, 1: +-40.96 mV
        uint16_t reserved3 : 4; // Reserved bits
    };

    // ADC Config register structure
    struct ADCConfigRegister {
        MeasurementMode mode : 4; // Measurement mode
        ConversionTime convTimeBusVoltage : 3; // Conversion time for bus voltage
        ConversionTime convTimeShuntVoltage : 3; // Conversion time for shunt voltage
        ConversionTime convTimeTemperature : 3;
        Averaging averaging : 3; // Averaging mode
    };

    // Diagnostic Alert register structure
    struct DiagnosticAlertRegister {
        uint16_t aLatch : 1;      // Bit 15: (Setting) Alert Latch Enable
        uint16_t cnvr : 1;        // Bit 14: (Setting) Conversion Ready on ALERT pin
        uint16_t slowAlert : 1;   // Bit 13: (Setting) ALERT comparison on averaged value
        uint16_t aPol : 1;        // Bit 12: (Setting) Alert Polarity (open-drain), 0 = Normal (active Low), 1 = Inverted (active High)
        uint16_t reserved1 : 2;   // Bits 11-10: Reserved, always read 0
        uint16_t mathOf : 1;      // Bit 9: Math Overflow Error
        uint16_t reserved2 : 1;   // Bit 8: Reserved, always read 0
        uint16_t tmPol : 1;       // Bit 7: Temperature Over-Limit Event
        uint16_t shntOl : 1;      // Bit 6: Shunt Over-Limit Event
        uint16_t shntUl : 1;      // Bit 5: Shunt Under-Limit Event
        uint16_t busOl : 1;       // Bit 4: Bus Over-Limit Event
        uint16_t busUl : 1;       // Bit 3: Bus Under-Limit Event
        uint16_t pol : 1;         // Bit 2: Power Over-Limit Event
        uint16_t cnvrf : 1;       // Bit 1: Conversion is Complete Flag
        uint16_t memStat : 1;     // Bit 0: Memory Checksum Error Status
    };

public:
    INA238(uint8_t address = 0x40);

    bool begin(TwoWire &wirePort = Wire);
    bool reset();

    bool setConfigRegister(ConfigRegister config);
    bool setADCConfigRegister(ADCConfigRegister adcConfig);
    bool setShuntCalibration(uint16_t calibrationValue);
    bool setDiagnosticAlertRegister(DiagnosticAlertRegister diagAlert);
    bool setShuntOvervoltageThreshold(float threshold);
    bool setShuntUndervoltageThreshold(float threshold);
    bool setBusOvervoltageThreshold(float threshold);
    bool setBusUndervoltageThreshold(float threshold);
    bool setTemperatureOverlimitThreshold(float temperature);
    bool setPowerOverLimitThreshold(float power);

    bool setShuntValues(float shuntResistance, float maxExpectedCurrent);

    bool readConfigRegister(ConfigRegister &config);
    bool readADCConfigRegister(ADCConfigRegister &adcConfig);
    bool readShuntCalibration(uint16_t &calibrationValue);
    bool readDiagnosticAlertRegister(DiagnosticAlertRegister &diagAlert);
    bool readShuntOvervoltageThreshold(float &threshold);
    bool readShuntUndervoltageThreshold(float &threshold);
    bool readBusOvervoltageThreshold(float &threshold);
    bool readBusUndervoltageThreshold(float &threshold);
    bool readTemperatureOverlimitThreshold(float &temperature);
    bool readPowerOverLimitThreshold(float &power);

    bool readBusVoltage(float &voltage);
    bool readShuntVoltage(float &voltage);
    bool readCurrent(float &current);
    bool readPower(float &power);
    bool readTemperature(float &temperature);

    bool readDiagnosticAlert(uint16_t &alert);

    


};
#endif // INA238_H