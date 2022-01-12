/************************************************************************
//The Class TCS34725_Color_Sensor.h is based upon modifications made to the Adafruit Library Adafruit_TCS34725
*/
/**************************************************************************/
/*!
@file     Adafruit_TCS34725.cpp
@author   KTOWN (Adafruit Industries)
@license  BSD (see license.txt)

Driver for the TCS34725 digital color sensors.

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

@section  HISTORY

v1.0 - First release
*/
/**************************************************************************/

#include "Arduino.h"
#include <Wire.h>
#include "TCS34725_Color_Sensor.h"

void TCS34725_I2C_ColorSensor::Setup(TCS34725_IntegrationTime integrationTime, TCS34725_RGBCGain colorGain)
{
	mIntegrationTimeCounter = millis();

	I2CWrite8Bits(TCS34725_COLOR_SENSOR_ATIME_REGISTER, (uint8_t)integrationTime);
	I2CWrite8Bits(TCS34725_COLOR_SENSOR_CONTROL_REGISTER, (uint8_t)colorGain);

	// set power on the chip on, after turning chip on must wait at least 2.4ms before turning on the RGBC
	I2CWrite8Bits(TCS34725_COLOR_SENSOR_ENABLE_REGISTER, TCS34725_COLOR_SENSOR_PON);
	delay(3);
	I2CWrite8Bits(TCS34725_COLOR_SENSOR_ENABLE_REGISTER, TCS34725_COLOR_SENSOR_PON | TCS34725_COLOR_SENSOR_AEN);
	mCurrentIntegrationTime = integrationTime;
	mIntegrationTime = (256 - (uint8_t)mCurrentIntegrationTime) * 2.4;
}

void TCS34725_I2C_ColorSensor::SetIntegrationTime(TCS34725_IntegrationTime integrationTime)
{
	mCurrentIntegrationTime = integrationTime;
	mIntegrationTime = (256 - (uint8_t)mCurrentIntegrationTime) * 2.4;
	I2CWrite8Bits(TCS34725_COLOR_SENSOR_ATIME_REGISTER, (uint8_t)integrationTime);
}

TCS34725_IntegrationTime TCS34725_I2C_ColorSensor::GetIntegrationTime()
{
	return static_cast<TCS34725_IntegrationTime>(I2CRead8Bits(TCS34725_COLOR_SENSOR_ATIME_REGISTER));
}

void TCS34725_I2C_ColorSensor::SetColorGain(TCS34725_RGBCGain colorGain)
{
	I2CWrite8Bits(TCS34725_COLOR_SENSOR_CONTROL_REGISTER, (uint8_t)colorGain);
}

TCS34725_RGBCGain TCS34725_I2C_ColorSensor::GetColorGain()
{
	return static_cast<TCS34725_RGBCGain>(I2CRead8Bits(TCS34725_COLOR_SENSOR_CONTROL_REGISTER) & 0x03);
}

void TCS34725_I2C_ColorSensor::Read()
{
	if (millis() - mIntegrationTimeCounter >= mIntegrationTime) // Only allows the color values to be updated once every integration cycle
	{

		mClear = I2CRead16Bits(TCS34725_COLOR_SENSOR_CLEAR_DATA_LOW_REGISTER);
		mRed = I2CRead16Bits(TCS34725_COLOR_SENSOR_RED_DATA_LOW_REGISTER);
		mGreen = I2CRead16Bits(TCS34725_COLOR_SENSOR_GREEN_DATA_LOW_REGISTER);
		mBlue = I2CRead16Bits(TCS34725_COLOR_SENSOR_BLUE_DATA_LOW_REGISTER);
		mIntegrationTimeCounter = millis();
	}
}

uint8_t TCS34725_I2C_ColorSensor::I2CRead8Bits(uint8_t reg)
{
	uint8_t retVal;
	Wire.beginTransmission(TCS34725_COLOR_SENSOR_ADDRESS); // signal a transmission begining to slave device
	Wire.write(TCS34725_COLOR_SENSOR_COMMAND_BIT | reg);   // Indicate which register to read from
	Wire.endTransmission();								   // transmits the write request to the color sensor

	Wire.requestFrom(TCS34725_COLOR_SENSOR_ADDRESS, 1); // requests 1 byte from the slave
	retVal = Wire.read();
}

uint16_t TCS34725_I2C_ColorSensor::I2CRead16Bits(uint8_t reg)
{
	uint16_t readLowByte, readHighByte;					   // signal a transmission begining to slave device
	Wire.beginTransmission(TCS34725_COLOR_SENSOR_ADDRESS); // Indicate which register to read from
	Wire.write(TCS34725_COLOR_SENSOR_COMMAND_BIT | reg);   // transmits the write request to the color sensor
	Wire.endTransmission();

	Wire.requestFrom(TCS34725_COLOR_SENSOR_ADDRESS, 2); // requests 2 byte from the slave
	readLowByte = Wire.read();
	readHighByte = Wire.read();
	readHighByte <<= 8;
	readHighByte |= readLowByte; // combine the high and low bytes to get the full color value
	return readHighByte;
}

void TCS34725_I2C_ColorSensor::I2CWrite8Bits(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(TCS34725_COLOR_SENSOR_ADDRESS); // signal a transmission begining to slave device
	Wire.write(TCS34725_COLOR_SENSOR_COMMAND_BIT | reg);   // Indicate which register to write to
	Wire.write(value);									   // transmit value to the requested register
	Wire.endTransmission();
}