#include <ESP32Servo.h>

#define PIN_SERVO2 13
#define PIN_SERVO3 14
#define PIN_SERVO4 15

Servo sg902;
Servo sg903;
Servo sg904;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  sg902.attach(PIN_SERVO2);
  sg903.attach(PIN_SERVO3);
  sg904.attach(PIN_SERVO4);
}

void loop() {
  // put your main code here, to run repeatedly:
  sg902.write(30);
  sg903.write(30);
  sg904.write(30);
}
