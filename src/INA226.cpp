/**
 * @file INA226.cpp
 * @author Seurat Atreides (https://github.com/seurat-atreides)
 * @brief Class 
 * @version 0.1
 * @date 2019-09-15
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "INA226.h"

/**
 * @brief               Initializes the I2C connection and sets the device address for later use
 * 
 * @param [in] address  Specifies the INA226 device address    
 */
void INA226::begin(uint8_t address) {
    Wire.begin();
    inaAddress = address;
} // of method begin

/**
 * @brief 
 * 
 * @param i2cSpeed 
 */
void INA226::setI2CSpeed(const uint32_t i2cSpeed ) {
  Wire.setClock(i2cSpeed);
} // of method setI2CSpeed

/**
 * @brief 
 * 
 * @param avg 
 * @param busConvTime 
 * @param shuntConvTime 
 * @param mode 
 * @return true  
 */
void INA226::configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode) {
    uint16_t config = 0;
    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);
    vBusMax = 36;
    vShuntMax = 0.08192f;
    writeRegister16(INA226_REG_CONFIG, config);
} // of method configure

/**
 * @brief 
 * 
 * @param calibrationValue 
 * @param rShuntValue 
 * @return true 
 * @return false 
 */
bool INA226::calibrate(uint16_t calibrationValue, float rShuntValue)
{
    //uint16_t calibrationValue;
    rShunt = rShuntValue;

    float iMaxPossible = vShuntMax / rShunt;

    currentLSB = iMaxPossible / 32768;

    //~ currentLSB = (uint16_t)(minimumLSB * 100000000);
    
    //~ currentLSB /= 100000000;
    //~ currentLSB /= 0.0001;
    //~ currentLSB = ceil(currentLSB);
    //~ currentLSB *= 0.0001;

    powerLSB = currentLSB * 25;

    //~ calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

    writeRegister16(INA226_REG_CALIBRATION, calibrationValue);

    return true;
}


/***************************************************************************************************************//*!
* @brief     will not return until the conversion for the specified device is finished
* @details   if no device number is specified it will wait until all devices have finished their current conversion. 
*            If the conversion has completed already then the flag (and interrupt pin, if activated) is also reset.
* @param[in] deviceNumber to reset (Optional, when not set then all devices have their mode changed)
*******************************************************************************************************************/
void INA226::waitForConversion(const uint8_t deviceNumber)
{
  uint16_t cvBits = 0;

  cvBits = 0;
  // Loop until the conversion is ready
  while(cvBits==0) {
    cvBits = readWord(INA226_REG_MASKENABLE,deviceNumber) & INA226_BIT_CVRF;
  } // of while the conversion not ready
} // of method waitForConversion

float INA226::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA226::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32768);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float INA226::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float INA226::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA226::readBusPower(void)
{
    return (readRegister16(INA226_REG_POWER) * powerLSB);
}

float INA226::readShuntCurrent(void)
{
    return (readRegister16(INA226_REG_CURRENT) * currentLSB);
}

float INA226::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA226_REG_SHUNTVOLTAGE);

    return (voltage * 0.0000025);
}

float INA226::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA226_REG_BUSVOLTAGE);

    return (voltage * 0.00125);
}

ina226_averages_t INA226::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina226_averages_t)value;
}

ina226_busConvTime_t INA226::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t INA226::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina226_shuntConvTime_t)value;
}

ina226_mode_t INA226::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina226_mode_t)value;
}

void INA226::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t INA226::getMaskEnable(void)
{
    return readRegister16(INA226_REG_MASKENABLE);
}

void INA226::enableShuntOverLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

/**
 * @brief 
 * 
 */
void INA226::enableShuntUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

void INA226::enableBusOvertLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

void INA226::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

void INA226::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

void INA226::enableConversionReadyAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

void INA226::setBusVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.00125;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setShuntVoltageLimit(float voltage)
{
    uint16_t value = voltage * 25000;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA226_BIT_APOL;
    } else
    {
        temp &= ~INA226_BIT_APOL;
    }

    setMaskEnable(temp);
}

void INA226::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA226_BIT_LEN;
    } else
    {
        temp &= ~INA226_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool INA226::isMathOverflow(void)
{
    return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool INA226::isAlert(void)
{
    return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}

int16_t INA226::readRegister16(uint8_t reg)
{
    int16_t value;

    Wire.beginTransmission(inaAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    delay(1);

    Wire.beginTransmission(inaAddress);
    Wire.requestFrom(inaAddress, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void INA226::writeRegister16(uint8_t reg, uint16_t val)
{
    uint8_t vla;
    vla = (uint8_t)val;
    val >>= 8;

    Wire.beginTransmission(inaAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write((uint8_t)val);
        Wire.write(vla);
    #else
        Wire.send(reg);
        Wire.send((uint8_t)val);
        Wire.send(vla);
    #endif
    Wire.endTransmission();
}
