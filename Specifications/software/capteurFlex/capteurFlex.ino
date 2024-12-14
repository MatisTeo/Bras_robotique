#include <ESP32Servo.h>

// Définition des broches
#define FLEX_SENSOR_PIN 39 // Nouvelle broche pour le capteur flex
#define SERVO_PIN_5 4      


// Définition des plages de mesure
const int FLEX_MIN = 3500;   // Lecture ADC minimale lorsque le capteur est détendu
const int FLEX_MAX = 3000;  // Lecture ADC maximale lorsque le capteur est complètement plié

const int SERVO_MIN_ANGLE = 0;   // Angle minimum du servo
const int SERVO_MAX_ANGLE = 90;  // Angle maximum du servo

Servo sg905; // Objet pour contrôler le servo

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Résolution 12 bits pour l'ADC

  // Configurer la broche du capteur flex
  pinMode(FLEX_SENSOR_PIN, INPUT);

  // Configurer le servo
  sg905.attach(SERVO_PIN_5);
  sg905.write(SERVO_MIN_ANGLE); // Position initiale à 0°
  
  delay(1000); // Pause pour stabiliser
}

void loop() {
  // Lire la valeur brute du capteur flex
  int flexValue = analogRead(FLEX_SENSOR_PIN);
  // Convertir la valeur du capteur flex en un angle servo
  int servoAngle = map(flexValue, FLEX_MIN, FLEX_MAX, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);

  // Limiter l'angle entre les bornes définies
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // Déplacer le servo à l'angle calculé
  sg905.write(servoAngle);

  // Afficher les valeurs pour le débogage
  Serial.print("Capteur Flex (ADC): ");
  Serial.print(flexValue);
  Serial.print(" | Angle Servo: ");
  Serial.println(servoAngle);

  delay(100); // Pause pour lisser les mouvements
}
