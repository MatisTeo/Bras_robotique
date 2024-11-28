#include <ESP32Servo.h>

#define PIN_SERVO 16  // Pin utilisée pour contrôler le servo moteur

Servo sg90;

void setup() {
  Serial.begin(115200);  // Initialisation de la communication série
  sg90.setPeriodHertz(50);  // Fréquence PWM standard pour les servos (50 Hz)
  
  sg90.attach(PIN_SERVO, 500, 2400); // Plage d'impulsions PWM (500µs - 2400µs)

  // Positionner le servo à 0° au départ
  sg90.write(45);
  delay(1000);  // Attendre 1 seconde
}

void loop() {
}
