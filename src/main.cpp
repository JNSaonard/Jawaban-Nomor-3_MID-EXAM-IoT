#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BH1750.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 4  // DHT11 sensor pin
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

BH1750 lightSensor;

#define LED_PIN 2    // Red LED pin

#define BAUD_RATE 9600 // Baud rate for UART communication

// WiFi credentials
const char* ssid = "javier";
const char* password = "javier01112003";

// MQTT broker details
const char* mqttBroker = "broker.emqx.io";
const char* mqttSubTopic = "Javier";
const char* mqttPubTopic = "Vier";
const char* mqttClientId = "ESP32Client";

// MQTT client and WiFi client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int x = 5; // Temperature reading delay in seconds
int y = 0; // Humidity reading delay in seconds
int z = 0; // Light sensor reading delay in seconds

void setup() {
  Serial.begin(BAUD_RATE); // initialize UART
  pinMode(LED_PIN, OUTPUT);
  
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Connect to MQTT broker
  mqttClient.setServer(mqttBroker, 1883);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT Broker...");
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("Failed to connect to MQTT Broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }

  Wire.begin(21,22);
  lightSensor.begin();
  dht.begin();
}

void loop() {
  // Read light sensor data
  uint16_t lux = lightSensor.readLightLevel();
  if (!isnan(lux)) {
    Serial.print("DATA Light: ");
    Serial.print(lux);
    Serial.println(" lux");
    if (lux > 400) {
      digitalWrite(LED_PIN, HIGH);
      char buffer[10];
      snprintf(buffer, 10, "%d", lux);
      mqttClient.publish(mqttPubTopic, ("WARNING, 2502002500 " + String(buffer) + " lux").c_str());
    } 
    if (lux < 400)
    {
      digitalWrite(LED_PIN, LOW);
      char buffer[10];
      snprintf(buffer, 10, "%d", lux);
      mqttClient.publish(mqttPubTopic, ("CLOSED, 2502002500 " + String(buffer) + " lux").c_str());
    }
  }
 // wait for Z seconds before sending the next light sensor reading
}
