#include "TCS34725_Color_Sensor.h"
#include <Wire.h>

#define TCAADDR 0x70
#define CSLAddr 0
#define CSRAddr 1
#define CSMAddr 2
class ColorSensor
{
public:
    uint16_t r, g, b, c;
    TCS34725_I2C_ColorSensor sensor;
    uint8_t addr;
    String tag;

    ColorSensor(String sensor_name, uint16_t multiplexer_addr)
    {
        r = 0;
        g = 0;
        b = 0;
        c = 0;
        addr = multiplexer_addr;
        tag = sensor_name;
    }

    // setup the sensor
    void setup()
    {
        tcaSelect(addr);
        sensor.Setup(TCS34725_IntegrationTime::INTEGRATION_TIME_125_MS, TCS34725_RGBCGain::GAIN_4_X);
    }

    // Reads the color sensor and stores the results
    void read()
    {
        tcaSelect(addr);

        sensor.Read();
        r = sensor.GetRed();
        g = sensor.GetGreen();
        b = sensor.GetBlue();
        c = sensor.GetClear();
    }

    // Prints the results
    void print()
    {
        Serial.print(tag + " ");
        Serial.print(r);
        Serial.print(" ");
        Serial.print(g);
        Serial.print(" ");
        Serial.print(b);
        Serial.print(" ");
        Serial.println(c);
    }

private:
    // Selects the multiplexer address
    static void tcaSelect(uint8_t addr)
    {
        Wire.beginTransmission(TCAADDR);
        Wire.write(1 << addr);
        Wire.endTransmission();
    }
};

ColorSensor colorSensorL = ColorSensor("Left", CSLAddr);
ColorSensor colorSensorR = ColorSensor("Right", CSRAddr);
ColorSensor colorSensorM = ColorSensor("Middle", CSMAddr);

void setup()
{
    Wire.begin();

    Serial.begin(9600);
    while (!Serial)
        ;

    colorSensorL.setup();
    colorSensorR.setup();
    colorSensorM.setup();
}

void loop()
{

    colorSensorL.read();
    colorSensorR.read();
    colorSensorM.read();

    colorSensorL.print();
    colorSensorR.print();
    colorSensorM.print();
    Serial.println();

    delay(3000);
}