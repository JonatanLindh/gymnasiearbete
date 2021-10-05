const int enPin = 10;
const int pin1 = 9;
const int pin2 = 12;

void setup()
{
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

void loop()
{
  go(true, 2);
  //delay(2000);
  //go(false, pin2);
  //delay(2000);
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