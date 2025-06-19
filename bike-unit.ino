#include <esp_now.h>
#include <WiFi.h>

#define RELAY_PIN 5  // GPIO pin for Relay
#define LED_PIN1 4   // GPIO pin for LED

typedef struct struct_message {
  bool helmetWorn;
  bool alcoholDetected;
  bool fallDetected;
  uint8_t checksum;
} struct_message;

struct_message myData;

// State variables
bool bikeAllowedToStart = false; // Initially, the bike cannot start

uint8_t calculateChecksum(struct_message data) {
  return data.helmetWorn + data.alcoholDetected + data.fallDetected;
}

// Corrected callback function signature
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // Validate checksum
  if (myData.checksum != calculateChecksum(myData)) {
    Serial.println("Checksum failed, data ignored.");
    return;  // Discard the data if checksum doesn't match
  }

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Helmet Worn: ");
  Serial.println(myData.helmetWorn);
  Serial.print("Alcohol Detected: ");
  Serial.println(myData.alcoholDetected);
  Serial.print("Fall Detected: ");
  Serial.println(myData.fallDetected);

  // Check conditions and update bike state
  if (myData.helmetWorn) {
    if (myData.alcoholDetected || myData.fallDetected) {
      Serial.println("Alcohol or fall detected, bike ignition off");
      bikeAllowedToStart = false;  // Bike should remain stopped
      digitalWrite(RELAY_PIN, LOW); // Ensure relay is off
      digitalWrite(LED_PIN1, LOW);  // Turn off the LED
    } else {
      bikeAllowedToStart = true; // Conditions are normal
      Serial.println("Helmet is worn, bike can be started");
    }
  } else {
    Serial.println("Helmet is not worn, bike cannot start");
    bikeAllowedToStart = false; // Bike should remain stopped
    digitalWrite(RELAY_PIN, LOW); //  relay off
    digitalWrite(LED_PIN1, LOW);  // Turn off the LED
  }

  // If conditions are normal, allow the bike to start
  if (bikeAllowedToStart) {
    digitalWrite(RELAY_PIN, HIGH); // Turn on the relay
    digitalWrite(LED_PIN1, HIGH);  // Turn on the LED
    Serial.println("Bike started");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(LED_PIN1, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // relay off initially

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onDataReceive);

  Serial.println("Bike Unit Initialized");
}

void loop() {
  
}
