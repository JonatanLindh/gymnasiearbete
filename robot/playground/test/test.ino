void setup()
{
  // pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop()
{
  go(true, 2);
  //delay(2000);
  //go(false, 9);
  //delay(2000);
}

void go(bool cc, int speed_control){
  int high, low;
  if (cc) {
    high = 9;
    low = 12;
  } else {
    high = 12;
    low = 9;
  }
  if (speed_control <= 3) {
    analogWrite(10, 250);
    digitalWrite(low, LOW);
    digitalWrite(high, HIGH);
  }
  digitalWrite(high, LOW);
  int speed = map(speed_control, 0, 9, 0, 255);
  analogWrite(10, speed);

  digitalWrite(low, LOW);
  digitalWrite(high, HIGH);
}
