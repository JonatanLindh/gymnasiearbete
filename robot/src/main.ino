#include "TCS34725_Color_Sensor.h"
#include <Wire.h>

#define cBlack 6500

//* Multiplexer settings
#define TCAADDR 0x70
#define CSLAddr 7
#define CSRAddr 2
#define CSMAddr 6

//* Motor Settings
#define MotorSpeed 200       //! 0-255
#define MotorSpeedSlower 118 //! 0-255

//* Left Motor Pins
#define MOTOR_L_EN_PIN 6 //! PWM
#define MOTOR_L_PIN1 46
#define MOTOR_L_PIN2 48

//* Right Motor Pins
#define MOTOR_R_EN_PIN 4 //! PWM
#define MOTOR_R_PIN1 42
#define MOTOR_R_PIN2 44

enum Direction : int
{
    Forward,
    Backward,
    Left,
    Right,
    Stop,
};

class ColorSensor
{
public:
    uint16_t r, g, b, c;
    boolean isBlack = false;
    boolean isGreen = false;
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
        sensor.Setup(TCS34725_IntegrationTime::INTEGRATION_TIME_24_MS, TCS34725_RGBCGain::GAIN_4_X);
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

        if (c < cBlack)
        {
            isBlack = true;
        }
        else
        {
            isBlack = false;
        }
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

class Motor
{
public:
    int enPin, pin1, pin2;

    Motor(int enPinNum, int pin1Num, int pin2Num)
    {
        enPin = enPinNum;
        pin1 = pin1Num;
        pin2 = pin2Num;
    }

    void setup()
    {
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
    }

    void start(Direction direction, int motor_speed)
    {
        int high, low;

        if (direction == Forward)
        {
            high = pin1;
            low = pin2;
        }

        else
        {
            high = pin2;
            low = pin1;
        }

        analogWrite(enPin, motor_speed);

        digitalWrite(low, LOW);
        digitalWrite(high, HIGH);
    }
    void stop()
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        analogWrite(enPin, 0);
    }
};

Motor leftMotor = Motor(MOTOR_L_EN_PIN, MOTOR_L_PIN1, MOTOR_L_PIN2);
Motor rightMotor = Motor(MOTOR_R_EN_PIN, MOTOR_R_PIN1, MOTOR_R_PIN2);

ColorSensor colorSensorL = ColorSensor("Left", CSLAddr);
ColorSensor colorSensorR = ColorSensor("Right", CSRAddr);
ColorSensor colorSensorM = ColorSensor("Middle", CSMAddr);

unsigned long time_black_reading = millis();

void setup()
{
    Wire.begin();
    Serial.begin(9600);

    colorSensorL.setup();
    colorSensorR.setup();
    colorSensorM.setup();

    leftMotor.setup();
    rightMotor.setup();
}

void loop()
{
    // Read all color sensors
    colorSensorL.read();
    colorSensorR.read();
    colorSensorM.read();

    // Print all color sensor values
    colorSensorL.print();
    colorSensorR.print();
    colorSensorM.print();
    Serial.println();

    delay(2000);

    // if (colorSensorL.isBlack || colorSensorM.isBlack || colorSensorR.isBlack)
    // {
    //     time_black_reading = millis();
    // }

    // if (!colorSensorL.isBlack && colorSensorM.isBlack && !colorSensorR.isBlack)
    // {
    //     leftMotor.start(Forward, MotorSpeed);
    //     rightMotor.start(Forward, MotorSpeed);
    // }
    // else if (colorSensorL.isBlack && !colorSensorM.isBlack)
    // {
    //     leftMotor.stop();
    //     rightMotor.start(Forward, MotorSpeedSlower);
    // }
    // else if (colorSensorL.isBlack && colorSensorM.isBlack)
    // {
    //     leftMotor.stop();
    //     rightMotor.start(Forward, MotorSpeedSlower + 30);
    // }
    // else if (!colorSensorM.isBlack, colorSensorR.isBlack)
    // {
    //     leftMotor.start(Forward, MotorSpeedSlower);
    //     rightMotor.stop();
    // }
    // else if (colorSensorM.isBlack, colorSensorR.isBlack)
    // {
    //     leftMotor.start(Forward, MotorSpeedSlower + 30);
    //     rightMotor.stop();
    // }
    // else if (!colorSensorL.isBlack && !colorSensorM.isBlack && !colorSensorR.isBlack && millis() - time_black_reading < 2000)
    // {
    //     leftMotor.start(Forward, MotorSpeed);
    //     rightMotor.start(Forward, MotorSpeed);
    // }
    // else
    // {
    //     leftMotor.stop();
    //     rightMotor.stop();
    // }
}