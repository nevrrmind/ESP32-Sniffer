//Mit Errorhandling und JSON->MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

// WLAN-Konfiguration
const char* ssid = "";
const char* password = "";

// SSIDs, die ignoriert werden sollen
const char* ignoredSSIDs[] = {"SSID1", "SSID2", "SSID3"}; // Füge hier die SSIDs hinzu, die ignoriert werden sollen
const int numIgnoredSSIDs = sizeof(ignoredSSIDs) / sizeof(ignoredSSIDs[0]);

// MAC-Adressen, die ignoriert werden sollen
const char* ignoredMACs[] = {"00:11:22:33:44:55", "AA:BB:CC:DD:EE:FF"}; // Füge hier die MAC-Adressen hinzu, die ignoriert werden sollen
const int numIgnoredMACs = sizeof(ignoredMACs) / sizeof(ignoredMACs[0]);

// RSSI-Schwellenwert
const int rssiThreshold = -90; // RSSI-Schwellenwert für die Filterung

// MQTT-Server-Konfiguration
const char* mqttServer = "";
const char* mqttTopic = "esp32/found_ssids";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectToWiFi() {
  Serial.println("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WLAN verbunden.");
}

void connectToMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);

  while (!mqttClient.connected()) {
    Serial.println("Verbinde mit MQTT...");

    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("MQTT verbunden.");
    } else {
      Serial.print("Verbindung zu MQTT fehlgeschlagen, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Versuche es in 5 Sekunden erneut.");
      delay(5000);
    }
  }
}

void publishData(const char* foundSSID, int rssi, const uint8_t* mac) {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  
  // Nur veröffentlichen, wenn SSID vorhanden ist und RSSI besser als der Schwellenwert ist
  if (strlen(foundSSID) > 0 && rssi > rssiThreshold) {
    StaticJsonDocument<256> doc;
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    doc["SSID"] = foundSSID;
    doc["RSSI"] = rssi;
    doc["MAC"] = macStr;

    char jsonOutput[256];
    serializeJson(doc, jsonOutput);

    mqttClient.publish(mqttTopic, jsonOutput);
  }
}

// Funktion zum Überprüfen, ob eine MAC-Adresse ignoriert werden soll
bool isMACIgnored(const char* mac) {
  for (int i = 0; i < numIgnoredMACs; i++) {
    if (strcmp(mac, ignoredMACs[i]) == 0) {
      Serial.print("Ignorierte MAC-Adresse gefunden: ");
      Serial.println(mac);
      return true;
    }
  }
  return false;
}

// Funktion zum Überprüfen, ob der RSSI-Wert besser als der Schwellenwert ist
bool isRSSIGood(int rssi) {
  if (rssi <= rssiThreshold) {
    Serial.print("Schlechter RSSI-Wert gefunden: ");
    Serial.println(rssi);
    return false;
  }
  return true;
}

// Callback-Funktion für den Empfang von Paketen
void snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  const wifi_promiscuous_pkt_t *promiscuousPacket = (wifi_promiscuous_pkt_t *)buf;
  const uint8_t *payload = promiscuousPacket->payload;
  const int rssi = promiscuousPacket->rx_ctrl.rssi;

  // MAC-Adresse des Senders extrahieren
  char addr[18];
  snprintf(addr, sizeof(addr), "%02X:%02X:%02X:%02X:%02X:%02X",
           payload[10], payload[11], payload[12],
           payload[13], payload[14], payload[15]);

  if (payload[0] == 0x40) { // Probe Request Frame prüfen
    // SSID-Länge befindet sich im 25ten Byte des Pakets
    int ssidLength = payload[25];
    if (ssidLength > 0 && ssidLength <= 32) { // Gültige SSID-Länge
      char foundSSID[33] = {0};
      memcpy(foundSSID, &payload[26], ssidLength); // SSID kopieren

      // Prüfen, ob die gefundene SSID in der Ignorierliste enthalten ist
      for (int i = 0; i < numIgnoredSSIDs; i++) {
        if (strcmp(foundSSID, ignoredSSIDs[i]) == 0) {
          Serial.print("Ignorierte SSID gefunden: ");
          Serial.println(foundSSID);
          // Gefundene SSID ist in der Ignorierliste, abbrechen
          return;
        }
      }

      // Prüfen, ob die MAC-Adresse ignoriert werden soll
      if (isMACIgnored(addr)) {
        return;
      }

      // Prüfen, ob der RSSI-Wert gut genug ist
      if (!isRSSIGood(rssi)) {
        return;
      }

      char message[100];
      snprintf(message, sizeof(message), "SSID: %s, RSSI: %d, MAC: %s", foundSSID, rssi, addr);
      Serial.println(message);
      
      publishData(foundSSID, rssi, payload+10); // +10, um die MAC-Adresse zu übergeben
    }
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  connectToMQTT();

  // Sniffer-Einstellungen
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(snifferCallback);
}

void loop() {
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("WLAN-Verbindung verloren, versuche erneut zu verbinden...");
    connectToWiFi();
  }
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop(); // Erlaubt dem MQTT-Client, eingehende Nachrichten zu verarbeiten und Verbindungen aufrechtzuerhalten
  // Keine weiteren Aktionen im Loop benötigt, da alle Aktionen im Callback und durch MQTT-Interaktionen ablaufen.
}
