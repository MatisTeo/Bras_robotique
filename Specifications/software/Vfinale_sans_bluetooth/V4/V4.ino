#include <ESP32Servo.h>

#define PIN_ACCEL_X 34
#define PIN_ACCEL_Y 35
#define PIN_ACCEL_Z 36
#define PIN_SERVO 16
#define PIN_SERVO2 13
#define PIN_SERVO3 14
#define PIN_SERVO4 15
#define FLEX_SENSOR_PIN 39 // Nouvelle broche pour le capteur flex
#define SERVO_PIN_5 4   

Servo sg90;
Servo sg902;
Servo sg903;
Servo sg904;
Servo sg905;

const int INITIAL_POSITION_BASE = 45;     // Position initiale du servo sg90
const int INITIAL_POSITION = 0;           // Position initiale des autres servos
const int MIN_ANGLE = 0;                  // Angle minimum
const int MAX_ANGLE = 90;                 // Angle maximum
const float ANGLE_CHANGE_THRESHOLD = 5.0; // Différence minimale pour un changement d'angle
const int FILTER_SAMPLES = 30;             // Nombre d'échantillons pour le filtrage
const int FLEX_MIN = 3500;   // Lecture minimale lorsque le capteur est détendu
const int FLEX_MAX = 3000;  // Lecture maximale lorsque le capteur est complètement plié
const int SERVO_MIN_ANGLE = 0;   // Angle minimum du servo pour la pince
const int SERVO_MAX_ANGLE = 90;  // Angle maximum du servo pour la pince

// Déclaration des variables pour le filtrage
float xHistory[FILTER_SAMPLES];
int filterIndex = 0;
float yHistory[FILTER_SAMPLES];

// Positions actuelles des servos
float currentAngleSg90 = INITIAL_POSITION_BASE;
float currentAngleSg902 = INITIAL_POSITION;
float currentAngleSg903 = INITIAL_POSITION;
float currentAngleSg904 = INITIAL_POSITION;
float currentAngleSg905 = SERVO_MIN_ANGLE;


void setup() {
  Serial.begin(115200);
  delay(1000);

  // Déclaration des PIN pour l'accéléromètre et les servos
  pinMode(PIN_ACCEL_X, INPUT);
  pinMode(PIN_ACCEL_Y, INPUT);
  pinMode(PIN_ACCEL_Z, INPUT);

  // On affecte chaque servo à son PIN
  sg90.attach(PIN_SERVO);
  sg902.attach(PIN_SERVO2);
  sg903.attach(PIN_SERVO3);
  sg904.attach(PIN_SERVO4);
  sg905.attach(SERVO_PIN_5);

  // On lit les résultats de l'accéléromètre sur 12 bits
  analogReadResolution(12);

  // Positionnement initial des servos
  sg90.write(INITIAL_POSITION_BASE);
  sg902.write(INITIAL_POSITION);
  sg903.write(INITIAL_POSITION);
  sg904.write(INITIAL_POSITION);
  sg905.write(SERVO_MIN_ANGLE);

  delay(1000);
}

// Fonction pour faire une moyenne de plusieurs échantillons en X afin d'avoir une position plus précise
float getFilteredX() {
  float currentX = analogRead(PIN_ACCEL_X);
  xHistory[filterIndex] = currentX;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;

  float sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += xHistory[i];
  }
  return sum / FILTER_SAMPLES;
}

// Fonction pour faire une moyenne de plusieurs échantillons en Y afin d'avoir une position plus précise
float getFilteredY() {
  float currentY = analogRead(PIN_ACCEL_Y);
  yHistory[filterIndex] = currentY;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;

  float sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += yHistory[i];
  }

  return sum / FILTER_SAMPLES;
}

void loop() {
  // Récupération des valeurs filtrées
  float currentX = getFilteredX();
  float currentY = getFilteredY();

  // Contrôle du servo moteur de la pince à l'aide du capteur flex

  // On récupère la valeur renvoyée par le capteur flex
  int flexValue = analogRead(FLEX_SENSOR_PIN);

  // Calcul de l'angle où on doit placer le servo en fonction des valeurs renvoyées par le capteur flex
  int servoAngle = map(flexValue, FLEX_MIN, FLEX_MAX, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);

  // On s'assure que l'angle calculé est compris entre 0 et 90 degré
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if (abs(servoAngle - currentAngleSg905) >= ANGLE_CHANGE_THRESHOLD) {
    // L'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent

    // On change la position du servo et on actualise la valeur de l'angle courant
      sg905.write(servoAngle);
      currentAngleSg905 = servoAngle;
  }

  // Contrôle du servo moteur en X

  // On map la valeur filtrée de l'accéléromètre en X pour convertir la valeur reçu en angle
  int mappedAngleSg90 = map(currentX, 1050, 700, 0, 90);

  // On s'assure que l'angle calculé est bien compris entre 0 et 90 degré
  mappedAngleSg90 = constrain(mappedAngleSg90, MIN_ANGLE, MAX_ANGLE);

  // On vérifie si la différence entre la nouvelle position et la position courante est d'au moins la valeur de ANGLE_CHANGE_THRESHOLD
  if (abs(mappedAngleSg90 - currentAngleSg90) >= ANGLE_CHANGE_THRESHOLD) {
    // La différence entre la nouvelle position et la position courante est d'au moins la valeur de ANGLE_CHANGE_THRESHOLD

    // On change la position du servo et on actualise la position courante
    currentAngleSg90 = mappedAngleSg90;
    sg90.write(currentAngleSg90);
  }

  // Contrôle des 3 servos moteurs en Y

  // On vérifie si la valeur de Y renvoyée par l'accéléromètre est inférieur à 1900 (valeur pour laquelle on veut que la pince soit perpendiculaire au sol)
  if (currentY < 850) {
    // La valeur en Y renvoyée par l'accéléromètre est inférieure à 1900

    // On met les 3 servos en position 0 si la position a variée d'au moins ANGLE_CHANGE_THRESHOLD
    sg902.write(0);
    sg903.write(0);
    sg904.write(0);

    // On actualise la valeur courante de l'angle pour chaque servo
    currentAngleSg902 = 0;
    currentAngleSg903 = 0;
    currentAngleSg904 = 0;
  } 
  // On vérifie si la valeur de Y renvoyée par l'accéléromètre est comprise entre 1900 et 2000 (valeurs pour lesquelles on veut que la pince commence à descendre)
  else if (currentY >= 850 && currentY < 900) {
    // La valeur de Y renvoyée par l'accéléromètre est comprise entre 1900 et 2000

    // On calcul l'angle qui correspondra à la nouvelle position du servo moteur du bas
    int angle2 = map(currentY, 850, 900, 0, 30);

    // On vérifie que l'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent
    if (abs(angle2 - currentAngleSg902) >= ANGLE_CHANGE_THRESHOLD) {
      // L'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent

      // On change la position du servo et on actualise la valeur de l'angle courant
      sg902.write(angle2);
      currentAngleSg902 = angle2;
    }

    // On laisse les deux autres servos en positions 0 qui correspond à leur valeur courante
    sg903.write(0);
    sg904.write(0);
    currentAngleSg903 = 0;
    currentAngleSg904 = 0;
  } 
  // On vérifie si la valeur de Y renvoyée par l'accéléromètre est comprise entre 2000 et 2100 (valeurs pour lesquelles on veut commencer à déplacer le deuxième servo moteur)
  else if (currentY >= 900 && currentY < 950) {
    // La valeur en Y renvoyée par l'accéléromètre est comprise entre 2000 et 2100

    // On calcul la nouvelle position du deuxième servo moteur
    int angle3 = map(currentY, 900, 950, 0, 30);

    // On vérifie que l'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent
    if (abs(angle3 - currentAngleSg903) >= ANGLE_CHANGE_THRESHOLD) {
      // L'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent

      // On change la position du servo et on actualise la valeur de l'angle courant
      sg903.write(angle3);
      currentAngleSg903 = angle3;
    }

    // On place les deux autres servo dans la position souhaitée et on ajuste la valeur de leur angle courant
    sg902.write(30);
    currentAngleSg902 = 30;
    sg904.write(0);
    currentAngleSg904 = 0;
  } 
  // On vérifie si la valeur de Y renvoyée par l'accéléromètre est comprise entre 2100 et 2200 (valeurs pour lesquelles on veut commencer à déplacer le troisième servo moteur)
  else if (currentY >= 950 && currentY < 1000) {
    // La valeur en Y renvoyée par l'accéléromètre est comprise entre 2100 et 2200

    // On calcul la nouvelle position du troisième servo moteur
    int angle4 = map(currentY, 950, 1000, 0, 30);

    // On vérifie que l'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent
    if (abs(angle4 - currentAngleSg904) >= ANGLE_CHANGE_THRESHOLD) {
      // L'angle varie d'au moins ANGLE_CHANGE_THRESHOLD par rapport à l'angle précédent
      sg904.write(angle4);

      // On change la position du servo et on actualise la valeur de l'angle courant
      currentAngleSg904 = angle4;
    }

    // On place les deux autres servo dans la position souhaitée et on ajuste la valeur de leur angle courant
    sg902.write(30);
    currentAngleSg902 = 30;
    sg903.write(30);
    currentAngleSg903 = 30;
  } 
  // On gère le cas où la valeur en Y renvoyée par l'accéléromètre est supérieure à 2200
  else {
    // On déplace tous les servos en position 30 et on actualise leur angle courant
    sg902.write(30);
    sg903.write(30);
    sg904.write(30);
    currentAngleSg902 = 30;
    currentAngleSg903 = 30;
    currentAngleSg904 = 30;
  }
  delay(10);
}



