#define MOTOR 10

void setup() {
  pinMode(MOTOR, OUTPUT);
}

void loop() {
  for (int i = 1; i <= 150; i++)
  {
    analogWrite(MOTOR, i);
    delay(50);
  }

  for (int i = 1; i <= 150; i++)
  {
    analogWrite(MOTOR, 150 - i);
    delay(50);
  }

  delay(1000);
}
