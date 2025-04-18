#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

#define MPU6050_ADDR 0x68  // Adresse I²C du MPU-6050
#define FLEX_SENSOR_PIN 39 // GPIO pour le capteur flex
#define BUTTON_PIN 18      // GPIO pour le bouton poussoir
#define FILTER_SAMPLES 30  // Nombre d'échantillons pour le filtrage

uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x6F, 0x06, 0x84};

// Historique pour filtrer les données des capteurs
float flexHistory[FILTER_SAMPLES];
int filterIndex = 0;

// Variables pour le debounce du bouton
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Structure pour envoyer les données des capteurs
struct SensorData {
    float accelX;
    float accelY;
    int flex;
    bool buttonState;
};
SensorData dataToSend;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Données envoyées avec succès" : "Échec de l'envoi des données");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Initialisation I²C
    
    // Configuration du bouton (simple INPUT au lieu de INPUT_PULLUP)
    pinMode(BUTTON_PIN, INPUT);
    
    // Vérifier la connexion avec le MPU-6050
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU-6050 non détecté !");
        while (1);
    }
    Serial.println("MPU-6050 détecté !");
    
    // Activer le MPU-6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Initialisation ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erreur d'initialisation ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    peerInfo.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Échec de l'ajout du pair");
        return;
    }
    
    analogReadResolution(12);
}

void readAccelData(int16_t *ax, int16_t *ay) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 4, true);
    
    if (Wire.available() == 4) {
        *ax = Wire.read() << 8 | Wire.read();
        *ay = Wire.read() << 8 | Wire.read();
    } else {
        Serial.println("Erreur de lecture MPU-6050 !");
    }
}

float getFilteredValue(float *history, int pin) {
    float current = analogRead(pin);
    history[filterIndex] = current;
    float sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) {
        sum += history[i];
    }
    filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
    return sum / FILTER_SAMPLES;
}

void loop() {
    int16_t ax, ay;
    readAccelData(&ax, &ay);
    
    // Lecture de l'état du bouton avec gestion du debounce
    int buttonReading = digitalRead(BUTTON_PIN);
    
    // Si l'état du bouton change, réinitialiser le timer de debounce
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // Mettre à jour l'état du bouton
        dataToSend.buttonState = (buttonReading == HIGH);
        lastDebounceTime = millis();
    }
    
    dataToSend.accelX = ax;
    dataToSend.accelY = ay;
    dataToSend.flex = getFilteredValue(flexHistory, FLEX_SENSOR_PIN);
    
    Serial.print("Bouton: "); 
    Serial.println(dataToSend.buttonState ? "Appuyé" : "Relâché");
    Serial.println(dataToSend.flex);
    esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
    
    delay(10); 
}