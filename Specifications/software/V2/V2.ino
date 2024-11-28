#include <ESP32Servo.h>

#define PIN_ACCEL_X 34
#define PIN_ACCEL_Y 35
#define PIN_ACCEL_Z 36
#define PIN_SERVO 16
#define PIN_SERVO2 13
#define PIN_SERVO3 14
#define PIN_SERVO4 15

Servo sg90;
Servo sg902;
Servo sg903;
Servo sg904;

const int INITIAL_POSITION_BASE = 45;     // Initialement, le servo sera en position 45
const int INITIAL_POSITION = 0;           // Initialement, le servo sera en position 0
const int MIN_ANGLE = 0;                  // On déclare l'angle minimum du servo
const int MAX_ANGLE = 90;                 // On déclare l'angle maximum du servo
const float STABILITY_THRESHOLD = 2.0;    // On déclare un seuil de stabilité pour ignorer les petits mouvements
const int FILTER_SAMPLES = 5;             // On déclare le nombre d'échantillons pour le filtrage

// Déclaration des variables pour le filtrage
float xHistory[FILTER_SAMPLES];
int filterIndex = 0;
float yHistory[FILTER_SAMPLES];

// Variables pour le calcul
float referenceX;  // Variable qui sert à déterminer la valeur de X sur l'accéléromètre
float referenceY;  // Valeur de référence pour Y
float currentServoAngle; // Position courante du servo moteur
bool isInitialized = false;  // Flag d'initialisation pour l'accéléromètre

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Déclaration des PIN pour l'accéléromètre et les servos
  pinMode(PIN_ACCEL_X, INPUT);
  pinMode(PIN_ACCEL_Y, INPUT);
  pinMode(PIN_ACCEL_Z, INPUT);

  // Attachement des servos aux PIN
  sg90.attach(PIN_SERVO);
  sg902.attach(PIN_SERVO2);
  sg903.attach(PIN_SERVO3);
  sg904.attach(PIN_SERVO4);

  // Lecture analogique sur 12 bits (valeur par défaut pour ESP32)
  analogReadResolution(12);

  // Positionnement initial des servos
  currentServoAngle = INITIAL_POSITION_BASE;
  sg90.write(INITIAL_POSITION_BASE);
  sg902.write(INITIAL_POSITION);
  sg903.write(INITIAL_POSITION);
  sg904.write(INITIAL_POSITION);
  delay(1000);

  // Calibration de l'accéléromètre
  calibrateAccelerometer();
}

// Méthode qui permet de calibrer l'accéléromètre
void calibrateAccelerometer() {
  float sumX = 0; // Variable pour lire les données remontées par le X de l'accéléromètre
  float sumY = 0; // Variable pour lire les données remontées par le Y de l'accéléromètre

  // Initialisation de l'historique des valeurs récupérées en X et Y
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    xHistory[i] = 0; // Historique des valeurs de X
  }

  // Récupération des 20 mesures pour X et Y
  for(int i = 0; i < 20; i++) {
    sumX += analogRead(PIN_ACCEL_X);  // Mesure de X
    sumY += analogRead(PIN_ACCEL_Y);  // Mesure de Y
    delay(50);  // Attente entre les mesures pour la stabilité
  }

  // Calcul des valeurs de référence pour X et Y
  referenceX = sumX / 20.0;
  referenceY = sumY / 20.0;

  // L'accéléromètre est maintenant calibré
  isInitialized = true;

  // Affichage des valeurs de référence
  Serial.print("Valeur X de référence: ");
  Serial.println(referenceX);
  Serial.print("Valeur Y de référence: ");
  Serial.println(referenceY);
  Serial.println("Calibration terminée - Vous pouvez maintenant incliner l'accéléromètre");
}

// Fonction qui permet de filtrer les échantillons remontés par l'accéléromètre pour X
float getFilteredX() {
  float currentX = analogRead(PIN_ACCEL_X);  // Récupération de la valeur courante de X

  // Ajout de la valeur à l'historique des valeurs pour X
  xHistory[filterIndex] = currentX;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES; // Garanti qu'on revient à 0 quand on atteint FILTER_SAMPLES

  // Calcul de la somme des échantillons pour X
  float sum = 0;
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    sum += xHistory[i];
  }

  return sum / FILTER_SAMPLES; // Renvoie la moyenne des échantillons pour X
}

// Fonction qui permet de filtrer les échantillons remontés par l'accéléromètre pour Y
float getFilteredY() {
  float currentY = analogRead(PIN_ACCEL_Y);  // Récupération de la valeur courante de Y

  // Ajout de la valeur à l'historique des valeurs pour Y
  yHistory[filterIndex] = currentY;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES; // Garanti qu'on revient à 0 quand on atteint FILTER_SAMPLES

  // Calcul de la somme des échantillons pour Y
  float sum = 0;
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    sum += yHistory[i];
  }

  return sum / FILTER_SAMPLES; // Renvoie la moyenne des échantillons pour Y
}

void loop() {
  // Si l'accéléromètre n'est pas initialisé, on sort de la boucle
  if (!isInitialized) return;

  // Récupération des valeurs filtrées de X et Y
  float currentX = getFilteredX();
  float currentY = getFilteredY();

  Serial.println(currentX);

  // Calcul de la différence en X par rapport à la valeur de référence
  float xDiff = currentX - referenceX;
  if(xDiff<0){
    xDiff=-xDiff;
  }

  // Conversion de la valeur X observée en une position pour le servo
  float maxDiff = 500;
  float angleOffset = (xDiff / maxDiff) * 45; // Conversion de la différence en angle

  // Calcul de la nouvelle position du servo
  float targetServoAngle = INITIAL_POSITION - angleOffset;
  targetServoAngle = constrain(targetServoAngle, MIN_ANGLE, MAX_ANGLE); // Limite l'angle entre MIN_ANGLE et MAX_ANGLE

  // Déplacement du servo si la différence dépasse le seuil de stabilité
  if (abs(targetServoAngle - currentServoAngle) > STABILITY_THRESHOLD) {
    currentServoAngle = targetServoAngle;
    sg90.write(round(currentServoAngle));

    Serial.print("Valeur X: ");
    Serial.print(currentX);
    Serial.print(" | Différence X: ");
    Serial.print(xDiff);
    Serial.print(" | Position servo: ");
    Serial.println(currentServoAngle);

    // Limites des positions des servos
    if (currentServoAngle <= MIN_ANGLE) {
      Serial.println("Limite minimum atteinte (0°)");
    } else if (currentServoAngle >= MAX_ANGLE) {
      Serial.println("Limite maximum atteinte (90°)");
    }
  }

  // Contrôle des servos 2, 3 et 4 en fonction de Y
  if (currentY < 1900) {
    sg902.write(0);
    sg903.write(0);
    sg904.write(0);
  } else if (currentY >= 1900 && currentY < 2000) {
    // Déplacement progressif de sg902
    int angle2 = map(currentY, 1900, 2000, 0, 30);
    sg902.write(angle2);
    sg903.write(0);
    sg904.write(0);
    //Serial.print("angle2:");
    //Serial.println(angle2);
  } else if (currentY >= 2000 && currentY < 2100) {
    // Déplacement progressif de sg903
    int angle3 = map(currentY, 2000, 2100, 0, 30);
    sg902.write(30);
    sg903.write(angle3);
    sg904.write(0);
    //Serial.print("angle3:");
    //Serial.println(angle3);
  } else if (currentY >= 2100 && currentY < 2200) {
    // Déplacement progressif de sg904
    int angle4 = map(currentY, 2100, 2200, 0, 30);
    sg904.write(angle4);
    sg903.write(30);
    sg902.write(30);
    //Serial.print("angle4:");
    //Serial.println(angle4);
  }

  delay(1000); // Attente avant la prochaine mesure
}
