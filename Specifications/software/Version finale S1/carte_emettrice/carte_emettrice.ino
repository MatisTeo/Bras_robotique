#include <esp_now.h>    
#include <WiFi.h>       

#define PIN_ACCEL_X 34        // Définit le GPIO utilisé pour lire les valeurs de l'accéléromètre sur l'axe X
#define PIN_ACCEL_Y 35        // Définit le GPIO utilisé pour lire les valeurs de l'accéléromètre sur l'axe Y
#define FLEX_SENSOR_PIN 39    // Définit le GPIO pour lire les valeurs du capteur flex
#define FILTER_SAMPLES 30     // Définit la taille de l'historique pour le filtrage des données

uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x6F, 0x06, 0x84}; // Adresse MAC du destinataire pour la transmission ESP-NOW (adresse MAC de la carte réceptrice)

// Historique pour filtrer les données des capteurs d'accélération
float xHistory[FILTER_SAMPLES];
float yHistory[FILTER_SAMPLES];
float flexHistory[FILTER_SAMPLES];
int filterIndex = 0; // Index pour parcourir les historiques

// Structure pour envoyer les données des capteurs
struct SensorData {
    float accelX; // Valeur filtrée de l'accéléromètre sur l'axe X
    float accelY; // Valeur filtrée de l'accéléromètre sur l'axe Y
    int flex;     // Valeur du capteur flex
};
SensorData dataToSend; // Instance de la structure utilisée pour l'envoi des données

// Fonction appelée lorsque les données sont envoyées pour savoir si l'envoie est réussi ou non
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Données envoyées avec succès" : "Échec de l'envoi des données");
}

void setup() {
    Serial.begin(115200); // Initialisation de la communication série à 115200 bauds
    WiFi.mode(WIFI_STA);  // Configuration du mode WiFi 

    // Initialisation d'ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erreur d'initialisation ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent); // Enregistre la fonction qui sera appelée à chaque tentative d'envoie de données via ESP-NOW

    esp_now_peer_info_t peerInfo; // Structure contenant les informations sur la carte réceptrice
    memcpy(peerInfo.peer_addr, broadcastAddress, 6); // On copie l'adresse MAC de la carte réceptrice
    peerInfo.channel = 0;                            // On définit sur quel canal Wifi on va communiquer
    peerInfo.encrypt = false;                        // On désactive le chiffrement des données envoyées

    // Enregistre la carte réceptrice dans la liste des pairs ESP-NOW et met un message d'erreur si l'ajout échoue
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Échec de l'ajout du pair");
        return;
    }

    analogReadResolution(12); // On configure l'ESP32 pour que son convertisseur analogique-numérique effectue des lectures avec une résolution de 12 bits
}

// Fonction pour filtrer les valeurs des capteurs en utilisant un historique
float getFilteredValue(float *history, int pin) {
    float current = analogRead(pin);      // Lit la valeur brute du capteur
    history[filterIndex] = current;       // Ajoute la nouvelle valeur à l'historique
    float sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++) { // Calcule la somme des valeurs dans l'historique
        sum += history[i];
    }
    filterIndex = (filterIndex + 1) % FILTER_SAMPLES; // L’opérateur modulo permet de boucler l’index lorsque filterIndex atteint la fin du tableau.
    return sum / FILTER_SAMPLES;                     // Retourne la moyenne des valeurs
}

void loop() {
    // Récupère les valeurs filtrées des accéléromètres
    dataToSend.accelX = getFilteredValue(xHistory, PIN_ACCEL_X);
    dataToSend.accelY = getFilteredValue(yHistory, PIN_ACCEL_Y);
    
    // Lit la valeur du capteur flex
    dataToSend.flex = getFilteredValue(flexHistory, FLEX_SENSOR_PIN);
    Serial.println(getFilteredValue(flexHistory, FLEX_SENSOR_PIN));

    // Envoie les données via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)); // effectue un cast pour convertir le pointeur vers dataToSend en un pointeur générique de type uint8_t *, qui est attendu par la fonction esp_now_send
    delay(10); 
}  
