#include <ESP32Servo.h>

#define PIN_ACCEL_X 34
#define PIN_ACCEL_Y 35
#define PIN_ACCEL_Z 36
#define PIN_SERVO 16

Servo sg90;
const int INITIAL_POSITION = 45;     // Initialement, le servo sera en position 45
const int MIN_ANGLE = 0;            // On déclare l'angle minimum du servo
const int MAX_ANGLE = 90;           // On déclare l'angle maximum du servo
const float STABILITY_THRESHOLD = 2.0; // On déclare un seuil de stabilité pour faire en sorte d'ignorer les petits mouvements
const int FILTER_SAMPLES = 5;       // On déclare le nombre d'échantillon pouir le filtrage pour que le servo moteur soit un peu moins sensible

// Déclaration des variables pour le filtrage
float xHistory[FILTER_SAMPLES];
int filterIndex = 0;

// Variables pour le calcul
float referenceX;                // Variable qui sert à déterminer la valeur de X sur l'accéléromètre au départ afin de calculer de combien on doit bouger le servo en fonction des nouvelles valeurs de X
float currentServoAngle;         // Variable pour stocker la position courante du servo moteur
bool isInitialized = false;      // Déclaration du flag d'initialisation pour savoir si l'accéléromètre s'est initialisé

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // On déclare qu'on veut récupérer les valeurs remontés par les PIN
  pinMode(PIN_ACCEL_X, INPUT);
  pinMode(PIN_ACCEL_Y, INPUT);
  pinMode(PIN_ACCEL_Z, INPUT);

  // On déclare quel PIN est utilisé pour le servo moteur
  sg90.attach(PIN_SERVO);

  // On lit les valeurs analogiques sur 12 bits (valeur par défaut pour l'ESP32)
  analogReadResolution(12);
  
  // On positionne le servo à sa position initiale
  currentServoAngle = INITIAL_POSITION;
  sg90.write(INITIAL_POSITION);
  delay(1000);

  // On appelle la fonction calibrateAccelerometer pour déterminer les valeurs de références de l'accéléromètre
  calibrateAccelerometer();
}

// Méthode qui permet de calibrer l'accéléromètre
void calibrateAccelerometer() {
  float sumX = 0; // Variable pour lire les données remontées par le X de l'accéléromètre
  
  // On initialise l'historique des valeurs récupérées en X à 0
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    xHistory[i] = 0;
  }
  
  // On récupère la somme de 20 mesures faites en X qui nous permettront de déterminer la valeur en X de référence
  for(int i = 0; i < 20; i++) {
    sumX += analogRead(PIN_ACCEL_X);
    delay(50);
  }
  
  // La valeur de référence en X est la moyenne est 20 mesures précédentes
  referenceX = sumX / 20.0;

  // On a maintenant initialisé l'accéléromètre en déterminant la valeur de référence en X
  isInitialized = true;
  
  // On affiche la valeur de référence déterminée
  Serial.print("Valeur X de référence: ");
  Serial.println(referenceX);
  Serial.println("Calibration terminée - Vous pouvez maintenant incliner l'accéléromètre");
}

// Fonction qui permet de filtrer les échantillons remontés par l'accélèromètre en renvoyant leur moyenne pour plus de précision car l'accéléromètre est trop sensible
float getFilteredX() {
  float currentX = analogRead(PIN_ACCEL_X);  // On récupère la valeur courante de X remontée par l'accéléromètre
  
  // On ajoute la valeur à l'historique des valeurs
  xHistory[filterIndex] = currentX;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES; // Garanti qu'on revient à 0 quand on récupère le nombre d'échantillon définit par FILTER_SAMPLES
  
  // On déclare une variable pour récupérer la somme des échantillons
  float sum = 0;

  // On récupère la somme des échantillons
  for(int i = 0; i < FILTER_SAMPLES; i++) {
    sum += xHistory[i];
  }

  // On renvoie la moyenne des échantillons
  return sum / FILTER_SAMPLES;
}

void loop() {
  // Si l'accéléromètre n'est pas initialisé on sort de la boucle
  if (!isInitialized) return;

  // On récupère la valeur courante de X filtrée
  float currentX = getFilteredX();
  
  // On calcule la différence en X par rapport à la valeur de référence
  float xDiff = currentX - referenceX;
  
  // On converti la valeur observée en X par une position du servo moteur
  float maxDiff = 500; // On choisit la différence maximale pour laquelle on va atteindre les positions extrèmes (0 ou 90)
  float angleOffset = (xDiff / maxDiff) * 45; // On converti la différence récupérée en angle (±45° par rapport à la position initiale)
  
  // On calcule la nouvelle position du servo moteur
  float targetServoAngle = INITIAL_POSITION - angleOffset; 
  targetServoAngle = constrain(targetServoAngle, MIN_ANGLE, MAX_ANGLE); // constrain permet de s'assurer que la position reste dans les limites qu'on a fixé
  
  // Si la position varie d'une valeur plus élevée que le seuil de stabilité on change la position du servo moteur, évite que le servo tremble car il réajuste tout le temps la position pour des petits mouvements
  if (abs(targetServoAngle - currentServoAngle) > STABILITY_THRESHOLD) {
    // La différence de position est suffisamment grande donc on déplace le servo
    currentServoAngle = targetServoAngle;
    sg90.write(round(currentServoAngle)); // round permet d'arrondir la valeur calculée
    
    // On affiche des informations pour le test
    Serial.print("Valeur X: ");
    Serial.print(currentX);
    Serial.print(" | Différence X: ");
    Serial.print(xDiff);
    Serial.print(" | Position servo: ");
    Serial.println(currentServoAngle);
    
    // On affiche un message si les valeurs limites sont atteintes
    if (currentServoAngle <= MIN_ANGLE) {
      Serial.println("Limite minimum atteinte (0°)");
    } else if (currentServoAngle >= MAX_ANGLE) {
      Serial.println("Limite maximum atteinte (90°)");
    }
  }
  
  delay(50);
}