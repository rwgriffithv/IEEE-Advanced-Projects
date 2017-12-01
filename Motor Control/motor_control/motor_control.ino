#define MOTOR 

void setup() {
  pinMode(MOTOR, OUTPUT);
}

void loop() {
  for (i = 1; i <= 25; i++)
  {
    analogWrite(MOTOR, i);
    delay(50);
  }

  for (i = 1; i <= 25; i++)
  {
    analogWrite(MOTOR, 25 - i);
    delay(50);
  }

  delay(1000);
}
