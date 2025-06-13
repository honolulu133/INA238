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
/**
 * @file ina238.h
 * @brief Header file for INA238 current and power monitor class.
 *
 * This file defines the INA238 class, which provides methods to interact with the INA238
 * current and power monitor IC over I2C. It includes enumerations for configuration options,
 * register definitions, and methods for reading and writing to the device.
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

// Public methods for INA238 class
public:
    /**
     * @brief Constructor for INA238 class.
     * Initializes the INA238 with a default I2C address.
     * @param address I2C address of the INA238 (default is 0x40).
     */
    INA238(uint8_t address = 0x40);

    /**
     * @brief Destructor for INA238 class.
     * Cleans up resources used by the INA238 instance.
     */
    ~INA238() = default;

    /**
     * @brief Initializes the INA238 with the specified I2C wire port.
     * This method sets up the I2C communication and prepares the INA238 for operation.
     * Wire.begin() must be called before this method.
     * @param wirePort Reference to the TwoWire object (default is Wire).
     * @return true if initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wirePort = Wire);
    /**
     * @brief Resets the INA238 device.
     * This method resets the INA238 to its default state.
     * @return true if the reset is successful, false otherwise.
     */
    bool reset();

    /**
     * @brief Sets the configuration register of the INA238.
     * This method configures the INA238 with the specified settings.
     * @param config Configuration settings for the INA238.
     * @return true if the configuration is set successfully, false otherwise.
     */
    bool setConfigRegister(ConfigRegister config);
    /**
     * @brief Sets the ADC configuration register of the INA238.
     * This method configures the ADC settings of the INA238.
     * @param adcConfig ADC configuration settings for the INA238.
     * @return true if the ADC configuration is set successfully, false otherwise.
     */
    bool setADCConfigRegister(ADCConfigRegister adcConfig);
    /**
     * @brief Sets the diagnostic alert register of the INA238.
     * This method configures the diagnostic alert settings of the INA238.
     * @param diagAlert Diagnostic alert settings for the INA238.
     * @return true if the diagnostic alert register is set successfully, false otherwise.
     */
    bool setDiagnosticAlertRegister(DiagnosticAlertRegister diagAlert);
    /**
     * @brief Sets the shunt overvoltage threshold.
     * This method sets the threshold for shunt overvoltage detection.
     * Resolution:
     * ADCRANGE = 0 (163.84 mV) -> 5 uV/LSB
     * ADCRANGE = 1 (40.96 mV) -> 1.25 uV/LSB
     * Default value is 0x7FFF (maximum threshold).
     * If negative values are entered in this register, then a shunt voltage measurement of 0 V trips this alarm. When
     * using negative values for the shunt under and overvoltage thresholds be aware that the over voltage threshold
     * must be set to the larger (that is, less negative) of the two values.
     * @param threshold Shunt overvoltage threshold in volts.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setShuntOvervoltageThreshold(float threshold);
    /**
     * @brief Sets the shunt undervoltage threshold.
     * This method sets the threshold for shunt undervoltage detection.
     * Resolution:
     * ADCRANGE = 0 (163.84 mV) -> 5 uV/LSB
     * ADCRANGE = 1 (40.96 mV) -> 1.25 uV/LSB
     * Default value is 0x8000 (minimum threshold).
     * @param threshold Shunt undervoltage threshold in volts.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setShuntUndervoltageThreshold(float threshold);
    /**
     * @brief Sets the bus overvoltage threshold.
     * This method sets the threshold for bus overvoltage detection.
     * Resolution: 3.125 mV/LSB
     * Default value is 0x7FFF (maximum threshold, positive values only).
     * @param threshold Bus overvoltage threshold in volts.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setBusOvervoltageThreshold(float threshold);
    /**
     * @brief Sets the bus undervoltage threshold.
     * This method sets the threshold for bus undervoltage detection.
     * Resolution: 3.125 mV/LSB
     * Default value is 0x0000 (minimum threshold, positive values only).
     * @param threshold Bus undervoltage threshold in volts.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setBusUndervoltageThreshold(float threshold);
    /**
     * @brief Sets the temperature overlimit threshold.
     * This method sets the threshold for temperature overlimit detection.
     * Resolution: 0.125 °C/LSB
     * Default value is 0x7FF (maximum threshold).
     * @param temperature Temperature overlimit threshold in degrees Celsius.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setTemperatureOverlimitThreshold(float temperature);
    /**
     * @brief Sets the power overlimit threshold.
     * This method sets the threshold for power overlimit detection.
     * Resolution: 256 × Power LSB
     * (Power LSB = Current LSB * 0.2)
     * Default value is 0xFFFF (maximum threshold, positive values only).
     * @param power Power overlimit threshold in watts.
     * @return true if the threshold is set successfully, false otherwise.
     */
    bool setPowerOverLimitThreshold(float power);

    /**
     * @brief Sets the shunt values for the INA238.
     * This method sets the shunt resistance and maximum expected current for the INA238.
     * The shunt calibration value is calculated based on the provided parameters.
     * @param shuntResistance Shunt resistance in ohms.
     * @param maxExpectedCurrent Maximum expected current in amperes.
     * @return true if the shunt values are set successfully, false otherwise.
     */
    bool setShuntValues(float shuntResistance, float maxExpectedCurrent);

    /**
     * @brief Reads the configuration register of the INA238.
     * This method reads the current configuration settings from the INA238.
     * @param config Reference to a ConfigRegister structure to store the read configuration.
     * @return true if the configuration is read successfully, false otherwise.
     */
    bool readConfigRegister(ConfigRegister &config);
    /**
     * @brief Reads the ADC configuration register of the INA238.
     * This method reads the current ADC configuration settings from the INA238.
     * @param adcConfig Reference to an ADCConfigRegister structure to store the read ADC configuration.
     * @return true if the ADC configuration is read successfully, false otherwise.
     */
    bool readADCConfigRegister(ADCConfigRegister &adcConfig);
    /**
     * @brief Reads the shunt calibration value from the INA238.
     * This method reads the shunt calibration value, which is used to calculate current.
     * @param calibrationValue Reference to a 16-bit unsigned integer to store the read calibration value.
     * @return true if the calibration value is read successfully, false otherwise.
     */
    bool readShuntCalibration(uint16_t &calibrationValue);
    /**
     * @brief Reads the diagnostic alert register of the INA238.
     * This method reads the diagnostic alert settings from the INA238.
     * @param diagAlert Reference to a DiagnosticAlertRegister structure to store the read diagnostic alert settings.
     * @return true if the diagnostic alert register is read successfully, false otherwise.
     */
    bool readDiagnosticAlertRegister(DiagnosticAlertRegister &diagAlert);
    /**
     * @brief Reads the shunt overvoltage threshold.
     * This method reads the shunt overvoltage threshold value.
     * @param threshold Reference to a float to store the read shunt overvoltage threshold in volts.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readShuntOvervoltageThreshold(float &threshold);
    /**
     * @brief Reads the shunt undervoltage threshold.
     * This method reads the shunt undervoltage threshold value.
     * @param threshold Reference to a float to store the read shunt undervoltage threshold in volts.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readShuntUndervoltageThreshold(float &threshold);
    /**
     * @brief Reads the bus overvoltage threshold.
     * This method reads the bus overvoltage threshold value.
     * @param threshold Reference to a float to store the read bus overvoltage threshold in volts.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readBusOvervoltageThreshold(float &threshold);
    /**
     * @brief Reads the bus undervoltage threshold.
     * This method reads the bus undervoltage threshold value.
     * @param threshold Reference to a float to store the read bus undervoltage threshold in volts.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readBusUndervoltageThreshold(float &threshold);
    /**
     * @brief Reads the temperature overlimit threshold.
     * This method reads the temperature overlimit threshold value.
     * @param temperature Reference to a float to store the read temperature overlimit threshold in degrees Celsius.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readTemperatureOverlimitThreshold(float &temperature);
    /**
     * @brief Reads the power overlimit threshold.
     * This method reads the power overlimit threshold value.
     * @param power Reference to a float to store the read power overlimit threshold in watts.
     * @return true if the threshold is read successfully, false otherwise.
     */
    bool readPowerOverLimitThreshold(float &power);

    /**
     * @brief Reads the bus voltage from the INA238.
     * This method reads the bus voltage value from the INA238.
     * @param voltage Reference to a float to store the read bus voltage in volts.
     * @return true if the bus voltage is read successfully, false otherwise.
     */
    bool readBusVoltage(float &voltage);
    /**
     * @brief Reads the shunt voltage from the INA238.
     * This method reads the shunt voltage value from the INA238.
     * @param voltage Reference to a float to store the read shunt voltage in volts.
     * @return true if the shunt voltage is read successfully, false otherwise.
     */
    bool readShuntVoltage(float &voltage);
    /**
     * @brief Reads the current from the INA238.
     * This method reads the current value from the INA238.
     * The current is calculated based on the shunt voltage and resistance.
     * @param current Reference to a float to store the read current in amperes.
     * @return true if the current is read successfully, false otherwise.
     */
    bool readCurrent(float &current);
    /**
     * @brief Reads the power from the INA238.
     * This method reads the power value from the INA238.
     * The power is calculated based on the bus voltage and current.
     * @param power Reference to a float to store the read power in watts.
     * @return true if the power is read successfully, false otherwise.
     */
    bool readPower(float &power);
    /**
     * @brief Reads the temperature from the INA238.
     * This method reads the temperature value from the INA238.
     * The temperature is read in degrees Celsius.
     * @param temperature Reference to a float to store the read temperature in degrees Celsius.
     * @return true if the temperature is read successfully, false otherwise.
     */
    bool readTemperature(float &temperature);
    
// Private members for INA238 class
private:
    // Private methods for internal operations
    bool read16BitSignedRegister(uint8_t reg, int16_t &value);
    bool write16BitSignedRegister(uint8_t reg, int16_t value);
    bool read16BitUnsignedRegister(uint8_t reg, uint16_t &value);
    bool write16BitUnsignedRegister(uint8_t reg, uint16_t value);
    bool read24BitUnsignedRegister(uint8_t reg, uint32_t &value);

    // Private member variables
    uint8_t _address; // I2C address of the INA238
    TwoWire *_wire;   // Pointer to the I2C wire object

    float _shuntResistance; // Shunt resistance value in ohms
    float _maxExpectedCurrent; // Maximum expected current in amperes
    float _lsbCurrent; // LSB current value in amperes
    float _lsbPower; // LSB power value in watts
    ADCRange _adcRange; // ADC range setting
    
};
#endif // INA238_H