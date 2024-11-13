#include <WiFi.h>
#include <DHT.h>
#include <NewPing.h>
#include <HX711.h>
#include <MQUnifiedsensor.h>

#define DHTPIN 15
#define DHTTYPE DHT22

// Ultrasonic sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 18
#define MAX_DISTANCE 200

// Gas sensor pins (example: MQ-7 CO sensor)
#define CO_SENSOR_PIN 34

// HX711 load cell sensor pins
#define DOUT 32
#define CLK 33

// Energy and Voltage sensor pins
#define ENERGY_PIN 36
#define VOLTAGE_PIN 39

// Relay control pin
#define RELAY_PIN 23  // GPIO pin connected to the relay module

// Wi-Fi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Create sensor objects
DHT dht(DHTPIN, DHTTYPE);
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
HX711 scale;
MQUnifiedsensor mqSensor(CO_SENSOR_PIN, "MQ-7");

// Variables for sensor data
float temperature = 0.0;
float humidity = 0.0;
long distance = 0;
float weight = 0.0;
float coLevel = 0.0;
float energyUsage = 0.0;
float voltage = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();

  // Initialize HX711
  scale.begin(DOUT, CLK);
  
  // Initialize gas sensor (MQ-7)
  mqSensor.init();

  // Set relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Initially turn off relay

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  delay(2000); // Allow WiFi connection to stabilize
}

void loop() {
  // Read data from DHT sensor (Temperature and Humidity)
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Read data from ultrasonic sensor (Distance)
  distance = sonar.ping_cm();

  // Read weight data from HX711 load cell
  weight = scale.get_units(10);

  // Read CO level from MQ sensor
  coLevel = mqSensor.readSensor();

  // Read energy usage (for example, use analog input)
  energyUsage = analogRead(ENERGY_PIN);
  
  // Read voltage (for example, use analog input)
  voltage = analogRead(VOLTAGE_PIN);

  // Print sensor data to Serial Monitor
  Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C ");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.print(" % ");
  Serial.print("Distance: "); Serial.print(distance); Serial.print(" cm ");
  Serial.print("Weight: "); Serial.print(weight); Serial.print(" g ");
  Serial.print("CO Level: "); Serial.print(coLevel); Serial.print(" ppm ");
  Serial.print("Energy Usage: "); Serial.print(energyUsage); Serial.print(" mA ");
  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");

  // Pollution control logic (CO level too high)
  if (coLevel > 100) { // Example threshold for CO level
    Serial.println("CO level too high! Activating ventilation...");
    digitalWrite(RELAY_PIN, HIGH);  // Turn on the relay (activate ventilation)
  }
  else {
    digitalWrite(RELAY_PIN, LOW);   // Turn off the relay (deactivate ventilation)
  }

  // Energy usage control logic (high energy usage)
  if (energyUsage > 500) { // Example threshold for high energy usage
    Serial.println("High energy usage detected! Reducing energy consumption...");
    digitalWrite(RELAY_PIN, LOW);   // Turn off the relay to reduce power usage (turn off equipment)
  }

  delay(2000); // Wait for 2 seconds before reading again
}
