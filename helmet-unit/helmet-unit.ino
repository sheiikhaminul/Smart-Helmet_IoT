#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#define TINY_GSM_MODEM_SIM800L
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>


// Pin definitions
#define FSR_PIN 36     //  FSR
#define TILT_PIN 15    //  Mercury Tilt Switch
#define LED_PIN 4      // GPIO pin for LED
#define LED_PIN2 13    // GPIO pin for LED
#define LED_PIN3 12    // GPIO pin for LED
#define MQ3_PIN 34    //  MQ-3 Alcohol Sensor
#define BUTTON_PIN 32  // Push Button

#define MODEM_TX 25
#define MODEM_RX 33


// Pin configuration for LEDs
#define LED_ROAD_HUMP 14
#define LED_ACCIDENT_PRONE 27
#define LED_ROAD_QUALITY 26

// Phone number to send SMS
#define PHONE_NUMBER "+8801849652311"

// Define serial connections
SoftwareSerial SerialAT(MODEM_TX, MODEM_RX);
// GPS Setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // TX2, RX2 for GPS
// Configure TinyGSM and TinyGPS++ libraries
TinyGsm modem(SerialAT);


// MPU6050 instance
MPU6050 mpu;

// SD Card Setup
#define SD_CS_PIN 5 // SD card CS pin
File roadFile;// SD Card Setup


int threshold = 700;           // Threshold for FSR sensor
float alcoholThreshold = 2.0;  // Threshold for MQ-3 sensor

uint8_t broadcastAddress[] = {0xD4, 0x8A, 0xFC, 0x60, 0x9A, 0x2C};  //  bike unit's MAC address

// Distances for geofence and alert
const float geofenceDistance = 0.1;  // 100 meters
const float alertDistance = 0.05;    // 50 meters
const float bearingThreshold = 30.0; // 30-degree,filtering based on heading

// Grid size (0.005 degrees)
const float gridSize = 0.005;

typedef struct struct_message {
  bool helmetWorn;
  bool alcoholDetected;
  bool fallDetected;
  uint8_t checksum;
} struct_message;

struct_message myData;
bool fallDetected = false;
unsigned long fallDetectionTime = 0;

// Global character array for storing GPS location
char location[50];
String latitudeStr, longitudeStr;

// Variables to ensure SMS is sent only once per event
bool alcoholSMSsent = false;
bool fallSMSsent = false;

uint8_t calculateChecksum(struct_message data) {
  return data.helmetWorn + data.alcoholDetected + data.fallDetected;
}


// Struct to store each point's data
struct Point {
  float lat;
  float lon;
  int roadHump;
  int accidentProne;
  int roadQuality;
  int gridLat;  // New field for grid latitude
  int gridLon;  // New field for grid longitude
};


// Maximum number of points in the dataset
#define MAX_POINTS 100
Point points[MAX_POINTS]; // Array to hold the dataset points
int pointCount = 0;       // Number of points in the dataset

// Helper function to calculate distance between two coordinates (Haversine formula)
float distance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // Radius of the Earth in km
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c;
  return d;
}

// Helper function to calculate the bearing between two coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  return fmod((bearing + 360.0), 360.0); // Normalize to 0-360 degrees
}

// Function to determine if a point is within the bearing threshold of the current heading
bool isWithinBearing(float currentBearing, float pointBearing) {
  float diff = fabs(currentBearing - pointBearing);
  return (diff <= bearingThreshold || diff >= (360 - bearingThreshold));
}

// Function to determine the grid cluster for a given coordinate
void getGridCluster(float lat, float lon, int &gridLat, int &gridLon) {
  gridLat = int(lat / gridSize);
  gridLon = int(lon / gridSize);
}

// Load the dataset from the SD card into memory
void loadDataset() {
  roadFile = SD.open("/road.txt");
  if (!roadFile) {
    Serial.println("Error: Unable to open road.txt");
    return;
  }

  roadFile.seek(0); // Reset file to the beginning
  roadFile.readStringUntil('\n'); // Skip the first row (header)

  while (roadFile.available() && pointCount < MAX_POINTS) {
    String line = roadFile.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      points[pointCount].lat = line.substring(0, line.indexOf(',')).toFloat();
      line = line.substring(line.indexOf(',') + 1);
      points[pointCount].lon = line.substring(0, line.indexOf(',')).toFloat();
      line = line.substring(line.indexOf(',') + 1);
      points[pointCount].roadHump = line.substring(0, line.indexOf(',')).toInt();
      line = line.substring(line.indexOf(',') + 1);
      points[pointCount].accidentProne = line.substring(0, line.indexOf(',') + 1).toInt();
      line = line.substring(line.indexOf(',') + 1);
      points[pointCount].roadQuality = line.toInt();

      // Calculate and store grid coordinates
      getGridCluster(points[pointCount].lat, points[pointCount].lon, points[pointCount].gridLat, points[pointCount].gridLon);

      pointCount++;
    }
  }

  roadFile.close();

  Serial.print("Loaded ");
  Serial.print(pointCount);
  Serial.println(" points from the dataset.");
}


// Function to check GPS location against the dataset in memory and control LEDs
void checkRoadConditions(float currentLat, float currentLon, float currentBearing) {
  float nearestDist = geofenceDistance;
  bool pointInRange = false;  // Flag to check if any point is within range

  // Struct to store the nearest point
  struct NearestPoint {
    float lat;
    float lon;
    int roadHump;
    int accidentProne;
    int roadQuality;
    float dist;
    float bearing;
  };

  NearestPoint nearestPoint;
  bool nearestSet = false;

  int currentGridLat, currentGridLon;
  getGridCluster(currentLat, currentLon, currentGridLat, currentGridLon);

  // Print current grid information
  Serial.print("Current Grid: (");
  Serial.print(currentGridLat);
  Serial.print(", ");
  Serial.print(currentGridLon);
  Serial.println(")");

  // Geofence logic - find points within 100 meters
  for (int i = 0; i < pointCount; i++) {
    // Check if the point is in the same grid cluster using pre-calculated grid coordinates
    if (points[i].gridLat == currentGridLat && points[i].gridLon == currentGridLon) {
      Serial.print("Point in Same Grid - Lat: ");
      Serial.print(points[i].lat, 6);
      Serial.print(", Lon: ");
      Serial.print(points[i].lon, 6);
      Serial.println();

      // Calculate the distance between the current location and the point
      float dist = distance(currentLat, currentLon, points[i].lat, points[i].lon);

      if (dist <= geofenceDistance) {
        // Calculate the bearing for this point
        float pointBearing = calculateBearing(currentLat, currentLon, points[i].lat, points[i].lon);

        // Check if the point is within the heading threshold
        if (isWithinBearing(currentBearing, pointBearing)) {
          pointInRange = true;  // We found a valid point

          if (!nearestSet || dist < nearestDist) {
            nearestPoint = {points[i].lat, points[i].lon, points[i].roadHump, points[i].accidentProne, points[i].roadQuality, dist, pointBearing};
            nearestDist = dist;
            nearestSet = true;
          }

          // Print the point within the geofence and heading range
          Serial.print("Geofence & Heading Point - Lat: ");
          Serial.print(points[i].lat, 6);
          Serial.print(", Lon: ");
          Serial.print(points[i].lon, 6);
          Serial.print(" | Distance: ");
          Serial.print(dist, 2);
          Serial.print(" km | Bearing: ");
          Serial.print(pointBearing);
          Serial.print(" | Road Hump: ");
          Serial.print(points[i].roadHump);
          Serial.print(" | Accident Prone: ");
          Serial.print(points[i].accidentProne);
          Serial.print(" | Road Quality: ");
          Serial.println(points[i].roadQuality);
        }
      }
    }
  }

  // Turn on LEDs for the nearest point within alert range (50 meters)
  if (pointInRange && nearestDist <= alertDistance) {
    digitalWrite(LED_ROAD_HUMP, nearestPoint.roadHump ? HIGH : LOW);
    digitalWrite(LED_ACCIDENT_PRONE, nearestPoint.accidentProne ? HIGH : LOW);
    digitalWrite(LED_ROAD_QUALITY, nearestPoint.roadQuality ? HIGH : LOW);

    
  // Calculate the heading error
   double headingError = fabs(nearestPoint.bearing - currentBearing);
    if (headingError > 180) {
     headingError = 360 - headingError;
    }

  // Print the nearest point information including the heading error
    Serial.print("Alert!! (Nearest point) - Lat: ");
    Serial.print(nearestPoint.lat, 6);
    Serial.print(", Lon: ");
    Serial.print(nearestPoint.lon, 6);
    Serial.print(" | Distance: ");
    Serial.print(nearestPoint.dist, 2);
    Serial.print(" km | Bearing Diff: ");
    Serial.println(headingError, 2);  // Print the heading error
  } else {
    // No valid points found, turn off all LEDs
    Serial.println("No valid points in range. All alert turned off.");
    digitalWrite(LED_ROAD_HUMP, LOW);
    digitalWrite(LED_ACCIDENT_PRONE, LOW);
    digitalWrite(LED_ROAD_QUALITY, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  SerialAT.begin(9600);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // GPS connected to TX2=16, RX2=17
  
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
   // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Load the dataset into memory
  loadDataset();

  pinMode(FSR_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(MQ3_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Set the button pin as input with internal pull-up resistor

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Adding delay to ensure GSM module initializes properly
  delay(3000);

  Serial.println("Initializing GSM module...");
  if (!modem.restart()) {
    Serial.println("Failed to restart GSM modem. Check power supply and connections.");
  } else {
    String modemInfo = modem.getModemInfo();
    Serial.print("Modem Info: ");
    Serial.println(modemInfo);
  }

  Serial.println("Helmet Detection System Initialized");

  // Initialize LED pins
  pinMode(LED_ROAD_HUMP, OUTPUT);
  pinMode(LED_ACCIDENT_PRONE, OUTPUT);
  pinMode(LED_ROAD_QUALITY, OUTPUT);

  // Turn off all LEDs initially
  digitalWrite(LED_ROAD_HUMP, LOW);
  digitalWrite(LED_ACCIDENT_PRONE, LOW);
  digitalWrite(LED_ROAD_QUALITY, LOW);

}

void readGPSAndCheckConditions() {
  // Simulate GPS reading from the GPS module
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      float currentLat = gps.location.lat();
      float currentLon = gps.location.lng();
      float currentBearing = gps.course.deg(); // Use GPS heading (in degrees)

      // Print current location and bearing
      Serial.print("Current Location - Lat: ");
      Serial.print(currentLat, 6);
      Serial.print(", Lon: ");
      Serial.print(currentLon, 6);
      Serial.print(" | Heading: ");
      Serial.println(currentBearing);

      // Check road conditions for the current GPS location
      checkRoadConditions(currentLat, currentLon, currentBearing);
    }
  }
}

void sendSMS(const char* message) {
  Serial.print("Sending SMS: ");
  Serial.println(message);

  // Send SMS using AT commands
  SerialAT.print("AT+CMGF=1\r"); // Set SMS mode to text
  delay(1000);
  
  // Replace PHONE_NUMBER with the actual recipient number
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(PHONE_NUMBER);
  SerialAT.print("\"\r");
  delay(1000);
  
  // Send the message text
  SerialAT.print(message);
  delay(1000);
  
  // Send Ctrl+Z to send the SMS
  SerialAT.write(0x1A);
}

void getGPSLocation() {
  unsigned long start = millis();
  bool locationAvailable = false;

  while (millis() - start < 2000) {  // Allow up to 2 seconds to get the GPS location
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
      if (gps.location.isValid()) {
        latitudeStr = String(gps.location.lat(), 6);
        longitudeStr = String(gps.location.lng(), 6);
        locationAvailable = true;
        break;
      }
    }
    if (locationAvailable) break;
  }

  if (gps.location.isValid()) {
    sprintf(location, "Lat: %s, Long: %s", latitudeStr.c_str(), longitudeStr.c_str());
  } else {
    strcpy(location, "Location not available");
  }
}

void checkAlcoholAndSendSMS(float alcoholValue, float alcoholThreshold) {
  if (alcoholValue > alcoholThreshold && !alcoholSMSsent) {
    digitalWrite(LED_PIN2, HIGH);  // Turn on the LED
    myData.alcoholDetected = true;
    Serial.println("Alcohol detected!!");

    // Construct the map link
    String coord_string = latitudeStr + ',' + longitudeStr;
    String map_link = "https://www.google.com/maps/place/" + coord_string;
    
    // Prepare SMS message
    String message = "Alert: Rider is drunk. Please check their status immediately.\nLatitude: " + latitudeStr + "\nLongitude: " + longitudeStr + "\nLocation: " + map_link;
    sendSMS(message.c_str());
    alcoholSMSsent = true;  // Mark the SMS as sent
    delay(2000);
    Serial.println("SMS sent successfully");
    digitalWrite(LED_PIN2, LOW);
  } else {
    myData.alcoholDetected = alcoholValue > alcoholThreshold;
  }
}

void checkFallAndSendSMS(float roll, float pitch) {
  if ((abs(roll) > 45.0 || abs(pitch) > 45.0) && !fallSMSsent) {  // Example threshold values
    digitalWrite(LED_PIN3, HIGH);               // Turn on the LED
    Serial.println("Fall detected, probably accident. WAITING.......");

    fallDetectionTime = millis();

    // Wait for 4 seconds to check if the button is pressed
    while (millis() - fallDetectionTime < 4000) {
      if (digitalRead(BUTTON_PIN) == LOW) {  // Button is pressed
        Serial.println("Button pressed (NOT ACCIDENT). No SMS will be sent.");
        digitalWrite(LED_PIN3, LOW);  // Turn off the LED
        fallSMSsent = false;          // Reset fall detection flag
        return;                       // Exit the function to prevent further actions
      }
    }

    if (digitalRead(BUTTON_PIN) == HIGH) {  // Button not pressed
      myData.fallDetected = true;
      Serial.println("Accident detected!!");

      // Construct the map link
      String coord_string = latitudeStr + ',' + longitudeStr;
      String map_link = "https://www.google.com/maps/place/" + coord_string;
      
      // Prepare SMS message
      String message = "Alert: Rider might have had an accident. Please check their status immediately.\nLatitude: " + latitudeStr + "\nLongitude: " + longitudeStr + "\nLocation: " + map_link;
      sendSMS(message.c_str());
      fallSMSsent = true;  // Mark the SMS as sent
      delay(2000);
      Serial.println("SMS sent successfully");
      digitalWrite(LED_PIN3, LOW);
    }

    delay(1000);
  } else {
    myData.fallDetected = (abs(roll) > 45.0 || abs(pitch) > 45.0);
  }
}



void loop() {

  
  int fsrValue = analogRead(FSR_PIN);
  int tiltState = digitalRead(TILT_PIN);
  float alcoholValue = analogRead(MQ3_PIN) * (5.0 / 4096.0);  // Assuming 12-bit ADC resolution

  // Get accelerometer data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate roll and pitch
  float roll = atan2(ay, az) * 180 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Get GPS location every second
  getGPSLocation();

  // Print values
  Serial.print("FSR Value: ");
  Serial.print(fsrValue);
  Serial.print(" | Tilt Switch State: ");
  Serial.print(tiltState);
  Serial.print(" | Alcohol Value: ");
  Serial.print(alcoholValue);
  Serial.print(" | Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | ");
  Serial.println(location);

  // Check if helmet is worn
  if (tiltState == LOW && fsrValue > threshold) {
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED
    myData.helmetWorn = true;
    Serial.println("Helmet is worn");

    // Check alcohol detection
    checkAlcoholAndSendSMS(alcoholValue, alcoholThreshold);
    // Check fall detection based on tilt values
    checkFallAndSendSMS(roll, pitch);
    //check roads and send alert
    readGPSAndCheckConditions();
    //delay(1000);
    // Set checksum
    myData.checksum = calculateChecksum(myData);

    // Send data to bike unit
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off the LED
    digitalWrite(LED_ROAD_HUMP, LOW);
    digitalWrite(LED_ACCIDENT_PRONE, LOW);
    digitalWrite(LED_ROAD_QUALITY, LOW);
    myData.helmetWorn = false;
    myData.alcoholDetected = false;
    myData.fallDetected = false;
    alcoholSMSsent = false;  // Reset alcohol detection flag
    fallSMSsent = false;     // Reset fall detection flag

    // Set checksum
    myData.checksum = calculateChecksum(myData);

    // Send data to bike unit
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }

  delay(2000);  // Delay for stability

}
