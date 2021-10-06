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

  void go(bool cc, int speed_control)
  {
    int high, low;

    if (cc)
    {
      high = pin2;
      low = pin1;
    }

    else
    {
      high = pin1;
      low = pin2;
    }

    int speed = map(speed_control, 0, pin2, 0, 255);
    analogWrite(enPin, speed);

    digitalWrite(low, LOW);
    digitalWrite(high, HIGH);
  }
};

Motor leftMotor = Motor(11, 12, 13);
Motor rightMotor = Motor(3, 2, 4);

void setup()
{
  leftMotor.setup();
  rightMotor.setup();
}

void loop()
{
  leftMotor.go(false, 4);
  rightMotor.go(true, 4);
}