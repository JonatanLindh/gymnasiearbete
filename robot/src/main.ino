#include <Wire.h>
#include <Adafruit_TCS34725.h>

byte gammatable[256]; // our RGB -> eye-recognized gamma color

#define TCAADDR 0x70

void tca_select(uint8_t i)
{
    if (i > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

// Create an instance of the TCS34725 Sensor
enum sensors
{
    left_color,
    right_color,
};

Adafruit_TCS34725 color_sensors[] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X), Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X)};

void sensor_setup(sensors sensor)
{
    tca_select(sensor);
    if (color_sensors[sensor].begin())
    { // if the sensor starts correctly
        Serial.print("Found sensor: ");
        Serial.println(sensor); // print the happy message
    }
    else
    {                                                                   // if the sensor starts incorrectly
        Serial.println("No TCS34725 found ... check your connections"); // print the not so happy message
        while (1)
            ; // halt!
    }
}

uint16_t *tcs_read(sensors sensor)
{

    tca_select(sensor);

    uint16_t clear, red, green, blue;

    color_sensors[sensor].setInterrupt(false);
    delay(60);
    color_sensors[sensor]
        .getRawData(&red, &green, &blue, &clear);
    color_sensors[sensor].setInterrupt(true);
    color_sensors[sensor].getRawData(&red, &green, &blue, &clear);

    uint16_t crgb[] = {clear, red, green, blue};

    return crgb;
}

void setup()
{
    Serial.begin(9600);                // Sart serial comms @ 9600 (you can change this)
    Serial.println("Color View Test"); // Title info

    // Set up color sensors
    sensor_setup(left_color);
    sensor_setup(right_color);
}

void loop()
{

    uint16_t *crgb = tcs_read(left_color);

    uint16_t clear = crgb[0];
    uint16_t red = crgb[1];
    uint16_t green = crgb[2];
    uint16_t blue = crgb[3];

    Serial.print("C:\t");
    Serial.print(clear); // print color values
    Serial.print("\tR:\t");
    Serial.print(red);
    Serial.print("\tG:\t");
    Serial.print(green);
    Serial.print("\tB:\t");
    Serial.println(blue);

    // Figure out some basic hex code for visualization
    uint32_t sum = clear;
    float r, g, b;
    r = red;
    r /= sum;
    g = green;
    g /= sum;
    b = blue;
    b /= sum;
    r *= 256;
    g *= 256;
    b *= 256;

    if ((r / g > 0.85 && r / g <= 1.25) && (r / b > 0.85 && r / b <= 1.25) && (g / b > 0.85 && g / b <= 1.25))
    {
        if (clear <= 3000) // Ändra baserat på distans
        {
            Serial.println("Black");
        }
        else if (clear > 3000 && clear <= 14000) // Ändra baserat på distans
        {
            Serial.println("Silver");
        }
        else
        {
            Serial.println("White");
        }
    }
    else if (g / r > 1.4 && g / b > 1.4)
    {
        Serial.println("Green");
    }
    else if (r / g > 1.4 && r / b > 1.4)
    {
        Serial.println("Red");
    }
    Serial.println();
}
