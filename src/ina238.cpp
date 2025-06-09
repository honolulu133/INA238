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

bool INA238::setShuntOvervoltageThreshold(float threshold);
bool INA238::setShuntUndervoltageThreshold(float threshold);
bool INA238::setBusOvervoltageThreshold(float threshold);
bool INA238::setBusUndervoltageThreshold(float threshold);
bool INA238::setTemperatureOverlimitThreshold(float temperature);
bool INA238::setPowerOverLimitThreshold(float power);


bool INA238::setShuntValues(float shuntResistance, float maxExpectedCurrent)
{
    // Store the shunt resistance and maximum expected current
    _shuntResistance = shuntResistance;
    _maxExpectedCurrent = maxExpectedCurrent;

    // Calculate the shunt calibration value
    const float SCALE_FACTOR = 819.2 * 1000000.0; //(819.2 * 10^6)
    
    // Calculate the current LSB value based on the maximum expected current
    _lsbCurrent = Imax / 32768.0;

    // Calculate the shunt calibration value
    float rawCal = SCALE_FACTOR * Rshunt * currentLsb;

    // Ensure proper rounding and clamp to max 2^15-1
    uint16_t shuntCal = static_cast<uint16_t>(std::min<int64_t>((rawCal + 0.5), 32767));

    // Write the shunt calibration value to the SHUNT_CAL register
    return write16BitRegister(INA238_REG_SHUNT_CAL, shuntCal);
}


