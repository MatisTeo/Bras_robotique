#include <ESP32Servo.h>       
#include <esp_now.h>            
#include <WiFi.h>               

#define PIN_SERVO 16            // Définition de la broche pour le servo SG90
#define PIN_SERVO2 13           // Définition de la broche pour le servo SG902
#define PIN_SERVO3 14           // Définition de la broche pour le servo SG903
#define PIN_SERVO4 15           // Définition de la broche pour le servo SG904
#define SERVO_PIN_5 4           // Définition de la broche pour le servo SG905

Servo sg90;                    // Déclaration de l'objet servo pour SG90 (Servo contrôlé par l'axe X de l'accéléromètre)
Servo sg902;                   // Déclaration de l'objet servo pour SG902 (Servo du bas contrôlé par l'axe Y de l'accéléromètre)
Servo sg903;                   // Déclaration de l'objet servo pour SG903 (Servo du milieu contrôlé par l'axe Y de l'accéléromètre)
Servo sg904;                   // Déclaration de l'objet servo pour SG904 (Servo du haut contrôlé par l'axe Y de l'accéléromètre)
Servo sg905;                   // Déclaration de l'objet servo pour SG905 (Servo pour l'ouverture et la fermeture de la pince contrôlé par le capteur flex)

const int FLEX_MIN = 3400;      // Définition de la valeur minimale du capteur flex
const int FLEX_MAX = 2700;      // Définition de la valeur maximale du capteur flex
const int SERVO_MIN_ANGLE = 0; // Définition de l'angle minimum pour les servos
const int SERVO_MAX_ANGLE = 90;// Définition de l'angle maximum pour les servos
const int SERVO_MIN_ANGLE_FLEX = 20;// Définition de l'angle maximum pour le servo du capteur flex
const int ANGLE_CHANGE_THRESHOLD = 1; // Seuil à dépasser pour changer l'angle du servo dans certains cas

float currentAngleSg90 = 0;    // Variable pour suivre l'angle actuel du servo SG90
float currentAngleSg902 = 0;   // Variable pour suivre l'angle actuel du servo SG902
float currentAngleSg903 = 0;   // Variable pour suivre l'angle actuel du servo SG903
float currentAngleSg904 = 0;   // Variable pour suivre l'angle actuel du servo SG904
float currentAngleSg905 = 0;   // Variable pour suivre l'angle actuel du servo SG905

struct SensorData {            // Définition d'une structure pour contenir les données du capteur
    float accelX;              // Accéléromètre sur l'axe X
    float accelY;              // Accéléromètre sur l'axe Y
    int flex;                  // Valeur du capteur flex
};
SensorData receivedData;       // Déclaration de l'objet `receivedData` pour stocker les données reçues

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    SensorData receivedData;  // Déclaration d'une variable locale pour stocker les données reçues
    memcpy(&receivedData, incomingData, sizeof(receivedData)); // Copie les données reçues dans receivedData

    // Calcul de l'angle du servo pour le capteur flex
    int servoAngle = map(receivedData.flex, FLEX_MIN, FLEX_MAX, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE_FLEX); // Mapping de la valeur flex à un angle de servo
    servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE_FLEX, SERVO_MAX_ANGLE); // On vérifie que le résultat est dans la plage de données valide
    if (abs(servoAngle - currentAngleSg905) >= ANGLE_CHANGE_THRESHOLD) { // Vérification si l'angle a changé d'au moins 5 degré
        sg905.write(servoAngle); // Envoie la nouvelle valeur d'angle au servo SG905
        currentAngleSg905 = servoAngle; // Met à jour l'angle actuel de SG905
    }

    // Calcul de l'angle pour l'accéléromètre X
    int mappedAngleSg90 = map(receivedData.accelX, 1050, 700, 0, 90); // Mapping de la valeur de l'accéléromètre X à un angle de servo.
    mappedAngleSg90 = constrain(mappedAngleSg90, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE); // On vérifie que le résultat est dans la plage de données valide
    if (abs(mappedAngleSg90 - currentAngleSg90) >= ANGLE_CHANGE_THRESHOLD) { // Vérification si l'angle a changé d'au moins 5 degré
        sg90.write(mappedAngleSg90); // Envoie la nouvelle valeur d'angle au servo SG90
        currentAngleSg90 = mappedAngleSg90; // Met à jour l'angle actuel de SG90
    }

    // Autres mouvements en fonction de Y
    if (receivedData.accelY < 700) { // Si la valeur de Y est inférieure à 700
        sg902.write(0); // Met le servo SG902 à 0°.
        sg903.write(0); // Met le servo SG903 à 0°.
        sg904.write(0); // Met le servo SG904 à 0°.
        currentAngleSg902 = currentAngleSg903 = currentAngleSg904 = 0; // Met à jour les angles actuels
    } else if (receivedData.accelY < 800) { // Si la valeur de Y est inférieure à 800
        int angle2 = map(receivedData.accelY, 850, 900, 0, 30); // Mappe la valeur de Y à un angle entre 0 et 30°
        sg902.write(angle2); // Envoie l'angle au servo SG902
        sg903.write(0); // Met le servo SG903 à 0°.
        sg904.write(0); // Met le servo SG904 à 0°.
        currentAngleSg902 = angle2; // Met à jour l'angle actuel de SG902
        currentAngleSg903 = currentAngleSg904 = 0; // Met à jour les angles actuels de SG903 et SG904
    } else if (receivedData.accelY < 900) { // Si la valeur de Y est inférieure à 900
        int angle3 = map(receivedData.accelY, 900, 950, 0, 30); // Mappe la valeur de Y à un angle entre 0 et 30°
        sg903.write(angle3); // Envoie l'angle au servo SG903
        sg902.write(30); // Met le servo SG902 à 30°
        sg904.write(0); // Met le servo SG904 à 0°
        currentAngleSg903 = angle3; // Met à jour l'angle actuel de SG903
        currentAngleSg902 = 30; // Met à jour l'angle actuel de SG902
        currentAngleSg904 = 0; // Met à jour l'angle actuel de SG904
    } else if (receivedData.accelY < 1050) { // Si la valeur de Y est inférieure à 1050
        int angle4 = map(receivedData.accelY, 950, 1000, 0, 30); // Mappe la valeur de Y à un angle entre 0 et 30°
        sg904.write(angle4); // Envoie l'angle au servo SG904
        sg902.write(30); // Met le servo SG902 à 30°
        sg903.write(30); // Met le servo SG903 à 30°
        currentAngleSg904 = angle4; // Met à jour l'angle actuel de SG904
        currentAngleSg902 = currentAngleSg903 = 30; // Met à jour les angles actuels de SG902 et SG903
    } else { // Si la valeur de Y est supérieure ou égale à 1050
        sg902.write(30); // Met le servo SG902 à 30°
        sg903.write(30); // Met le servo SG903 à 30°
        sg904.write(30); // Met le servo SG904 à 30°
        currentAngleSg902 = currentAngleSg903 = currentAngleSg904 = 30; // Met à jour les angles actuels de SG902, SG903 et SG904
    }
}

void setup() {
    Serial.begin(115200);        
    WiFi.mode(WIFI_STA); // Configure l'ESP32 en mode WiFi

    // Initialisation d'ESP-NOW
    if (esp_now_init() != ESP_OK) { // Initialise ESP-NOW pour la communication sans fil
        Serial.println("Erreur d'initialisation ESP-NOW"); // Si l'initialisation échoue, afficher un message d'erreur
        return;
    }

    esp_now_register_recv_cb(OnDataRecv); // Enregistre la fonction qui sera appelée à chaque tentative d'envoie de données via ESP-NOW

    sg90.attach(PIN_SERVO);         // Attache le servo SG90 à la broche PIN_SERVO
    sg902.attach(PIN_SERVO2);       // Attache le servo SG902 à la broche PIN_SERVO2
    sg903.attach(PIN_SERVO3);       // Attache le servo SG903 à la broche PIN_SERVO3
    sg904.attach(PIN_SERVO4);       // Attache le servo SG904 à la broche PIN_SERVO4
    sg905.attach(SERVO_PIN_5);      // Attache le servo SG905 à la broche SERVO_PIN_5

    sg90.write(0);                 // Initialise le servo SG90 à 0°
    sg902.write(0);                // Initialise le servo SG902 à 0°
    sg903.write(0);                // Initialise le servo SG903 à 0°
    sg904.write(0);                // Initialise le servo SG904 à 0°
    sg905.write(0);                // Initialise le servo SG905 à 0°
}

void loop() {
    // Boucle principale vide car l'action est déclenchée par la réception de données via ESP-NOW
}

