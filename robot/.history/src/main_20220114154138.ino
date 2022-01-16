#include "TCS34725_Color_Sensor.h"
#include <Wire.h>

//* Multiplexer settings
#define TCAADDR 0x70
#define CSLAddr 6
#define CSRAddr 7
#define CSMAddr 2

//* Motor Settings
#define MotorSpeed 255       //! 0-255
#define MotorSpeedSlower 100 //! 0-255

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

        if (c < 4000 && r < 2000 && g < 2000 && b < 2000)
        {
            isBlack = true;
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

void drive(Direction direction)
{
    if (direction == Forward)
    {
        Serial.print("Going forward");
        rightMotor.start(Forward, MotorSpeed);
        leftMotor.start(Forward, MotorSpeed);
    }
    else if (direction == Backward)
    {
        Serial.print("Going backward");
        rightMotor.start(Backward, MotorSpeed);
        leftMotor.start(Backward, MotorSpeed);
    }
    else if (direction == Left)
    {
        Serial.print("Going left");
        rightMotor.start(Forward, MotorSpeed);
        leftMotor.start(Forward, MotorSpeedSlower);
    }
    else if (direction == Right)
    {
        Serial.print("Going right");
        rightMotor.start(Forward, MotorSpeedSlower);
        leftMotor.start(Forward, MotorSpeed);
    }
    else if (direction == Stop)
    {
        rightMotor.stop();
        leftMotor.stop();
    }
}

Motor leftMotor = Motor(MOTOR_L_EN_PIN, MOTOR_L_PIN1, MOTOR_L_PIN2);
Motor rightMotor = Motor(MOTOR_R_EN_PIN, MOTOR_R_PIN1, MOTOR_R_PIN2);

ColorSensor colorSensorL = ColorSensor("Left", CSLAddr);
ColorSensor colorSensorR = ColorSensor("Right", CSRAddr);
ColorSensor colorSensorM = ColorSensor("Middle", CSMAddr);

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

    if (!colorSensorL.isBlack && !colorSensorR.isBlack)
    {
        drive(Forward);
    }
    else if (colorSensorL.isBlack && !colorSensorR.isBlack)
    {
        drive(Right);
    }
    else if (!colorSensorL.isBlack && colorSensorR.isBlack)
    {
        drive(Left);
    }

    // Move the robot

    delay(3000);
}