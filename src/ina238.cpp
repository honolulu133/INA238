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

#include "ina238.h"



INA238::INA238(uint8_t address)
    : _address(address),
    _wire(nullptr),
    _shuntResistance(1.0f),
    _maxExpectedCurrent(1.0f),
    _lsbCurrent(1.0f / 32768.0f),
    _adcRange(ADCRange::RANGE_163_84_MV)
{
    // Constructor initializes the INA238 with the specified I2C address
}

bool INA238::begin(TwoWire &wirePort)
{
    // Store the I2C wire port
    _wire = &wirePort;

    // Reset the INA238 to ensure it is in a known state
    return reset();
}

bool INA238::reset()
{
    // Write the reset command to the RESET register
    return write16BitRegister(INA238_REG_RESET, 0x8000);
}

bool INA238::setConfigRegister(ConfigRegister config)
{
    // Write the configuration settings to the CONFIG register
    return write16BitRegister(INA238_REG_CONFIG, reinterpret_cast<const uint16_t &>(config));
}

bool INA238::setADCConfigRegister(ADCConfigRegister adcConfig)
{
    // Write the ADC configuration settings to the ADC_CONFIG register
    return write16BitRegister(INA238_REG_ADC_CONFIG, reinterpret_cast<const uint16_t &>(adcConfig));
}

bool INA238::setDiagnosticAlertRegister(DiagnosticAlertRegister diagAlert)
{
    // Write the diagnostic alert settings to the DIAG_ALERT register
    return write16BitRegister(INA238_REG_DIAG_ALERT, reinterpret_cast<const uint16_t &>(diagAlert));
}

bool INA238::setShuntOvervoltageThreshold(float threshold)
{
    // Clamp the threshold to the maximum and minimum value based on the ADC range
    if (_adcRange == ADCRange::RANGE_163_84_MV) {
        threshold = std::max(-0.16384f, std::min(threshold, 0.16384f));
    } else {
        threshold = std::max(-0.04096f, std::min(threshold, 0.04096f));
    }

    // Select the appropriate scale factor based on the ADC range
    float scaleFactor = (_adcRange == ADCRange::RANGE_163_84_MV) ? 5.0e-6f : 1.25e-6f; // 5 uV/LSB or 1.25 uV/LSB

    // Calculate the raw threshold value based on the scale factor
    int16_t rawThreshold = static_cast<int16_t>(threshold / scaleFactor);

    // Set the raw threshold value to the SHUNT_OVERVOLTAGE register
    return write16BitSignedRegister(INA238_REG_SHUNT_OVERVOLTAGE, rawThreshold);
}

bool INA238::setShuntUndervoltageThreshold(float threshold)
{
    // Clamp the threshold to the maximum and minimum value based on the ADC range
    if (_adcRange == ADCRange::RANGE_163_84_MV) {
        threshold = std::max(-0.16384f, std::min(threshold, 0.16384f));
    } else {
        threshold = std::max(-0.04096f, std::min(threshold, 0.04096f));
    }

    // Select the appropriate scale factor based on the ADC range
    float scaleFactor = (_adcRange == ADCRange::RANGE_163_84_MV) ? 5.0e-6f : 1.25e-6f; // 5 uV/LSB or 1.25 uV/LSB

    // Calculate the raw threshold value based on the scale factor
    int16_t rawThreshold = static_cast<int16_t>(threshold / scaleFactor);

    // Set the raw threshold value to the SHUNT_UNDERVOLTAGE register
    return write16BitSignedRegister(INA238_REG_SHUNT_UNDERVOLTAGE, rawThreshold);
}

bool INA238::setBusOvervoltageThreshold(float threshold)
{
    // Clamp the threshold to the maximum value (Vbus max = 85V)
    threshold = std::max(0.0f, std::min(threshold, 85.0f)); // Maximum bus voltage for INA238

    // Calculate the raw threshold value based on the scale factor
    int16_t rawThreshold = static_cast<int16_t>(threshold / 0.003125f);

    // Set the raw threshold value to the BUS_OVERVOLTAGE register
    return write16BitSignedRegister(INA238_REG_BUS_OVERVOLTAGE, rawThreshold);
}

bool INA238::setBusUndervoltageThreshold(float threshold)
{
    // Clamp the threshold to the minimum and maximum value
    threshold = std::max(0.0f, std::min(threshold, 32767.0f * 0.003125f)); // 3.125 mV/LSB

    // Calculate the raw threshold value based on the scale factor
    int16_t rawThreshold = static_cast<int16_t>(threshold / 0.003125f);

    // Set the raw threshold value to the BUS_UNDERVOLTAGE register
    return write16BitSignedRegister(INA238_REG_BUS_UNDERVOLTAGE, rawThreshold);
}

bool INA238::setTemperatureOverlimitThreshold(float temperature)
{
    // Clamp the temperature to the maximum (recommended) values
    temperature = std::max(-40.0f, std::min(temperature, 125.0f)); // Temperature range for INA238

    // Calculate the raw threshold value based on the scale factor
    int16_t rawThreshold = static_cast<int16_t>(temperature / 0.125f); // 0.125 °C/LSB

    // Set the raw threshold value to the TEMPERATURE_OVERLIMIT register
    return write16BitSignedRegister(INA238_REG_TEMPERATURE_OVERLIMIT, rawThreshold);
}

bool INA238::setPowerOverLimitThreshold(float power)
{
    // Clamp the power to the maximum value based on the current LSB
    power = std::max(0.0f, std::min(power, 32767.0f * _lsbPower * 256.0f)); // Power LSB = Current LSB * 0.2

    // Calculate the raw threshold value based on the scale factor
    uint16_t rawThreshold = static_cast<uint16_t>(power / _lsbPower / 256.0f); // 256 * Power LSB

    // Set the raw threshold value to the POWER_OVERLIMIT register
    return write16BitUnsignedRegister(INA238_REG_POWER_OVERLIMIT, rawThreshold);
}

bool INA238::setShuntValues(float shuntResistance, float maxExpectedCurrent)
{
    // Store the shunt resistance and maximum expected current
    _shuntResistance = shuntResistance;
    _maxExpectedCurrent = maxExpectedCurrent;

    // Calculate the shunt calibration value
    const float SCALE_FACTOR = 819.2 * 1000000.0; //(819.2 * 10^6)
    
    // Calculate the current LSB value based on the maximum expected current
    _lsbCurrent = Imax / 32768.0;

    // Calculate the power LSB value based on the current LSB
    _lsbPower = _lsbCurrent * 0.2;

    // Calculate the shunt calibration value
    float rawCal = SCALE_FACTOR * Rshunt * currentLsb;

    // Ensure proper rounding and clamp to max 2^15-1
    uint16_t shuntCal = static_cast<uint16_t>(std::min<int64_t>((rawCal + 0.5), 32767));

    // Write the shunt calibration value to the SHUNT_CAL register
    return write16BitRegister(INA238_REG_SHUNT_CAL, shuntCal);
}

bool INA238::readConfigRegister(ConfigRegister &config)
{
    // Read the CONFIG register and store the result in the provided ConfigRegister structure
    uint16_t rawValue;
    if (!read16BitUnsignedRegister(INA238_REG_CONFIG, rawValue)) {
        return false; // Failed to read the register
    }
    config = *reinterpret_cast<ConfigRegister *>(&rawValue);
    return true; // Successfully read the configuration register
}

bool INA238::readADCConfigRegister(ADCConfigRegister &adcConfig)
{
    // Read the ADC_CONFIG register and store the result in the provided ADCConfigRegister structure
    uint16_t rawValue;
    if (!read16BitUnsignedRegister(INA238_REG_ADC_CONFIG, rawValue)) {
        return false; // Failed to read the register
    }
    adcConfig = *reinterpret_cast<ADCConfigRegister *>(&rawValue);
    return true; // Successfully read the ADC configuration register
}

bool INA238::readShuntCalibration(uint16_t &calibrationValue)
{
    // Read the SHUNT_CAL register and store the result in the provided calibration value
    return read16BitUnsignedRegister(INA238_REG_SHUNT_CAL, calibrationValue);
}

bool INA238::readDiagnosticAlertRegister(DiagnosticAlertRegister &diagAlert)
{
    // Read the DIAG_ALERT register and store the result in the provided DiagnosticAlertRegister structure
    uint16_t rawValue;
    if (!read16BitUnsignedRegister(INA238_REG_DIAG_ALERT, rawValue)) {
        return false; // Failed to read the register
    }
    diagAlert = *reinterpret_cast<DiagnosticAlertRegister *>(&rawValue);
    return true; // Successfully read the diagnostic alert register
}

bool INA238::readShuntOvervoltageThreshold(float &threshold)
{
    // Read the SHUNT_OVERVOLTAGE register and convert the raw value to volts
    int16_t rawThreshold;
    if (!read16BitSignedRegister(INA238_REG_SHUNT_OVERVOLTAGE, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Select the appropriate scale factor based on the ADC range
    float scaleFactor = (_adcRange == ADCRange::RANGE_163_84_MV) ? 5.0e-6f : 1.25e-6f; // 5 uV/LSB or 1.25 uV/LSB

    // Convert the raw threshold to volts
    threshold = rawThreshold * scaleFactor;
    return true; // Successfully read the shunt overvoltage threshold
}

bool INA238::readShuntUndervoltageThreshold(float &threshold)
{
    // Read the SHUNT_UNDERVOLTAGE register and convert the raw value to volts
    int16_t rawThreshold;
    if (!read16BitSignedRegister(INA238_REG_SHUNT_UNDERVOLTAGE, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Select the appropriate scale factor based on the ADC range
    float scaleFactor = (_adcRange == ADCRange::RANGE_163_84_MV) ? 5.0e-6f : 1.25e-6f; // 5 uV/LSB or 1.25 uV/LSB

    // Convert the raw threshold to volts
    threshold = rawThreshold * scaleFactor;
    return true; // Successfully read the shunt undervoltage threshold
}

bool INA238::readBusOvervoltageThreshold(float &threshold)
{
    // Read the BUS_OVERVOLTAGE register and convert the raw value to volts
    int16_t rawThreshold;
    if (!read16BitSignedRegister(INA238_REG_BUS_OVERVOLTAGE, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Convert the raw threshold to volts (3.125 mV/LSB)
    threshold = rawThreshold * 0.003125f; // 3.125 mV/LSB
    return true; // Successfully read the bus overvoltage threshold
}

bool INA238::readBusUndervoltageThreshold(float &threshold)
{
    // Read the BUS_UNDERVOLTAGE register and convert the raw value to volts
    int16_t rawThreshold;
    if (!read16BitSignedRegister(INA238_REG_BUS_UNDERVOLTAGE, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Convert the raw threshold to volts (3.125 mV/LSB)
    threshold = rawThreshold * 0.003125f; // 3.125 mV/LSB
    return true; // Successfully read the bus undervoltage threshold
}

bool INA238::readTemperatureOverlimitThreshold(float &temperature)
{
    // Read the TEMPERATURE_OVERLIMIT register and convert the raw value to degrees Celsius
    int16_t rawThreshold;
    if (!read16BitSignedRegister(INA238_REG_TEMPERATURE_OVERLIMIT, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Convert the raw threshold to degrees Celsius (0.125 °C/LSB)
    temperature = rawThreshold * 0.125f; // 0.125 °C/LSB
    return true; // Successfully read the temperature overlimit threshold
}

bool INA238::readPowerOverLimitThreshold(float &power)
{
    // Read the POWER_OVERLIMIT register and convert the raw value to watts
    uint16_t rawThreshold;
    if (!read16BitUnsignedRegister(INA238_REG_POWER_OVERLIMIT, rawThreshold)) {
        return false; // Failed to read the register
    }

    // Convert the raw threshold to watts (256 * Power LSB)
    power = rawThreshold * _lsbPower * 256.0f; // Power LSB = Current LSB * 0.2
    return true; // Successfully read the power overlimit threshold
}
