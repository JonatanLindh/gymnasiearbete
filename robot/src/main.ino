#include "TCS34725_Color_Sensor.h"
#include <Wire.h>

#define TCAADDR 0x70
#define CSLAddr 0
#define CSRAddr 1
#define CSMAddr 2

TCS34725_I2C_ColorSensor colorSensorL;
TCS34725_I2C_ColorSensor colorSensorR;
TCS34725_I2C_ColorSensor colorSensorM;

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    while (!Serial)
        ;

    Serial.println("Setting Left Sensor to 125ms integration time and 4x gain");
    tcaSelect(CSLAddr);
    colorSensorL.Setup(TCS34725_IntegrationTime::INTEGRATION_TIME_125_MS, TCS34725_RGBCGain::GAIN_4_X); // Set up color sensor
    Serial.print("Integration Time: ");
    Serial.println((uint8_t)colorSensorL.GetIntegrationTime(), HEX);

    Serial.println("Setting Right Sensor to 125ms integration time and 4x gain");
    tcaSelect(CSRAddr);
    colorSensorR.Setup(TCS34725_IntegrationTime::INTEGRATION_TIME_125_MS, TCS34725_RGBCGain::GAIN_4_X); // Set up color sensor

    Serial.println("Setting Middle Sensor to 125ms integration time and 4x gain");
    tcaSelect(CSMAddr);
    colorSensorM.Setup(TCS34725_IntegrationTime::INTEGRATION_TIME_125_MS, TCS34725_RGBCGain::GAIN_4_X); // Set up color sensor
}

void loop()
{
    // put your main code here, to run repeatedly:
    Serial.println("Testing Left Sensor");
    tcaSelect(CSLAddr);
    // delay(1);
    Serial.print("Integration Time: ");
    Serial.println((uint8_t)colorSensorL.GetIntegrationTime(), HEX);

    ReadColorSensor("COLORL", colorSensorL);

    Serial.println("Testing Right Sensor");
    tcaSelect(CSRAddr);
    // delay(1);
    ReadColorSensor("COLORR", colorSensorR);

    Serial.println("Testing Middle Sensor");
    tcaSelect(CSMAddr);
    // delay(1);
    ReadColorSensor("COLORR", colorSensorM);
    delay(3000);
}

void tcaSelect(uint8_t addr)
{
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << addr);
    Wire.endTransmission();
}

//*******************************************
// Color Sensor code
//*******************************************
void ReadColorSensor(String tag, TCS34725_I2C_ColorSensor &colorSensor)
{
    uint16_t r, g, b, c;

    colorSensor.Read();
    r = colorSensor.GetRed();
    g = colorSensor.GetGreen();
    b = colorSensor.GetBlue();
    Serial.print(tag + " ");
    Serial.print(r);
    Serial.print(" ");
    Serial.print(g);
    Serial.print(" ");
    Serial.print(b);
    Serial.print(" ");
    Serial.println(c);
}