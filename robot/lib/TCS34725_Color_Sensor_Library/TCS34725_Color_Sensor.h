/************************************************************************
//The Class TCS34725_Color_Sensor.h is based upon modifications made to the Adafruit Library Adafruit_TCS34725
*/
/**************************************************************************/
/*!
@file     Adafruit_TCS34725.h
@author   KTOWN (Adafruit Industries)

@section LICENSE

Software License Agreement (BSD License)

Copyright (c) 2013, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#pragma once

#include "Arduino.h"


#define TCS34725_COLOR_SENSOR_ADDRESS 0x29
#define TCS34725_COLOR_SENSOR_COMMAND_BIT 0x80

#define TCS34725_COLOR_SENSOR_ENABLE_REGISTER 0x00
#define TCS34725_COLOR_SENSOR_ATIME_REGISTER 0x01
#define TCS34725_COLOR_SENSOR_CONTROL_REGISTER 0x0F
#define TCS34725_COLOR_SENSOR_CLEAR_DATA_LOW_REGISTER 0x14
#define TCS34725_COLOR_SENSOR_RED_DATA_LOW_REGISTER 0x16
#define TCS34725_COLOR_SENSOR_GREEN_DATA_LOW_REGISTER 0x18
#define TCS34725_COLOR_SENSOR_BLUE_DATA_LOW_REGISTER 0x1A

#define TCS34725_COLOR_SENSOR_PON 0x01
#define TCS34725_COLOR_SENSOR_AEN 0x02

//used to set the color integration time for the sensor
enum class TCS34725_IntegrationTime
{
	INTEGRATION_TIME_2_4_MS = 0xFF,
	INTEGRATION_TIME_24_MS = 0xF6,
	INTEGRATION_TIME_101_MS = 0xD5,
	INTEGRATION_TIME_125_MS = 0xCC,
	INTEGRATION_TIME_154_MS = 0xC0,
	INTEGRATION_TIME_700_MS = 0x00,
};

//used to set the color gain for the sensor
enum class TCS34725_RGBCGain
{
	GAIN_1_X = 0x00,
	GAIN_4_X = 0x01,
	GAIN_16_X = 0x02,
	GAIN_60_X = 0x03
};

class TCS34725_I2C_ColorSensor
{
public:

	/**
	* @brief default constructor
	*/
	TCS34725_I2C_ColorSensor():mRed(0),mGreen(0),mBlue(0),mClear(0) {};

	/**
	* @brief Setup method for an instance of the class.  Run this before reading values
	*
	* This method should be called in the Arduino's Setup() method.  It will set the TCS34725 to the power on state with the integration time and color gain specified
	* @param integrationTime TCS34725_IntegrationTime the amount of time the color sensor waits collecting color data before updating its internal registers
	* @param colorGain TCS34725_RGBCGain the amount of color amplification to be applied to the color values
	*/
	void Setup(TCS34725_IntegrationTime integrationTime, TCS34725_RGBCGain colorGain);

	/**
	* @brief sets the Integration Time
	*
	* @param integrationTime TCS34725_IntegrationTime the amount of time the color sensor waits collecting color data before updating its internal registers
	*/
	void SetIntegrationTime(TCS34725_IntegrationTime integrationTime);

	/**
	* @brief Gets the current Integration Time setting
	*
	* @return TCS34725_IntegrationTime  The current integration time value the sensor is set
	*/
	TCS34725_IntegrationTime GetIntegrationTime();

	/**
	* @brief Sets the color gain control
	*
	* @param colorGain TCS34725_RGBCGain the amount of color amplification to be applied to the color values
	*/
	void SetColorGain(TCS34725_RGBCGain colorGain);

	/**
	* @brief Gets the current color gain value
	*
	* @return TCS34725_RGBCGain The current color gain value the sensor is set
	*/
	TCS34725_RGBCGain GetColorGain();

	/**
	* @brief Reads the current RGBC values stored in the sensor's color registers
	*
	* This should be called in the Arduino's loop() method.  This will read the current color data from the sensor's registers.  The color data can then be retrieved by calling the Getter methods for the desired color
	* This method will only read once per Integration cycle.  The actual integration time in milliseconds is calculated using:
	* (256-ATIME)*2.4 = Integration Time
	* Where ATIME is the value residing in TCS34725_COLOR_SENSOR_ATIME_REGISTER
	*/
	void Read();

	/**
	* @brief Gets the current Red value 
	*
	* @return the current red color value
	*/
	uint16_t GetRed() { return mRed; }
	
	/**
	* @brief Gets the current Green value
	*
	* @return the current green color value
	*/
	uint16_t GetGreen() { return mGreen; }
	
	/**
	* @brief Gets the current Blue value
	*
	* @return the current blue color value
	*/
	uint16_t GetBlue() { return mBlue; }
	
	/**
	* @brief Gets the current Clear value
	*
	* @return the current Clear color value
	*/
	uint16_t GetClear() { return mClear; }

private:
	uint8_t I2CRead8Bits(uint8_t reg);

	uint16_t I2CRead16Bits(uint8_t reg);

	void I2CWrite8Bits(uint8_t reg, uint8_t value);



	uint16_t mRed, mGreen, mBlue, mClear;
	TCS34725_IntegrationTime mCurrentIntegrationTime;
	unsigned long mIntegrationTimeCounter;
	float mIntegrationTime;

};