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

  enum Direction : int
  {
    Clockwise,
    CounterClockwise,
  };

  void setup()
  {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void start(Direction direction, int speed_control)
  // void go(bool dir, int speed_control)
  {
    int high, low;

    if (direction == Clockwise)
    // if (dir)
    {
      high = pin1;
      low = pin2;
    }

    else
    {
      high = pin2;
      low = pin1;
    }

    int speed = map(speed_control, 0, 9, 0, 255);
    analogWrite(enPin, speed);

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

Motor leftMotor = Motor(11, 12, 13);
Motor rightMotor = Motor(3, 2, 4);

void setup()
{
  leftMotor.setup();
  rightMotor.setup();
}

void loop()
{
  rightMotor.start(Motor::Clockwise, 9);
  delay(1500);
  rightMotor.stop();

  leftMotor.start(Motor::Clockwise, 9);
  delay(1500);
  leftMotor.stop();
}