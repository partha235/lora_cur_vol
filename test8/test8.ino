/***********************************
Transmitter with ACS712, Voltage Sensor, DHT11, and LM35
***********************************/

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// Pin Definitions
#define CURRENT_SENSOR_PIN A0  // ACS712 connected to Analog Pin A0
#define VOLTAGE_SENSOR_PIN A1  // Voltage sensor connected to Analog Pin A1
#define DHT_PIN 2             // DHT11 data pin connected to Digital Pin 2
#define DHT_TYPE DHT11        // Specify DHT sensor type
#define TEMP_PROBE_PIN A2     // LM35 connected to Analog Pin A2

// Constants for Calibration
const float ACS712_SENSITIVITY = 0.185;  // Sensitivity for ACS712-05B (185mV/A)
const float VOLTAGE_SENSOR_RATIO = 11.0; // Voltage divider ratio (e.g., 100k:10k)
const float LM35_SCALING_FACTOR = 0.01;  // 10mV per degree Celsius for LM35

// Variables
int counter = 0;  // Packet counter
float current = 0.0;
float voltage = 0.0;
float temperature = 0.0;       // DHT11 Temperature
float humidity = 0.0;          // DHT11 Humidity
float probeTemperature = 0.0;  // LM35 Temperature

// DHT Sensor Object
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  // Initialize LoRa
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1); // Halt on failure
  }

  LoRa.setTxPower(20);
  Serial.println("LoRa Initialized");

  // Initialize DHT Sensor
  dht.begin();
}

void loop() {
  // Read Current from ACS712
  current = readACS712(CURRENT_SENSOR_PIN);

  // Read Voltage from Voltage Sensor
  voltage = readVoltageSensor(VOLTAGE_SENSOR_PIN);

  // Read Temperature and Humidity from DHT11
  temperature = dht.readTemperature();  // Read temperature in Celsius
  humidity = dht.readHumidity();        // Read relative humidity

  // Read Temperature from LM35 Probe
  probeTemperature = readLM35(TEMP_PROBE_PIN);

  // Check if DHT11 readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 0.0;
    humidity = 0.0;
  }

  // Send Data
  Serial.print("Sending packet: ");
  Serial.println(counter);

  Serial.print("Current: ");
  Serial.print(current, 2);
  Serial.print(" A, Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V, DHT Temp: ");
  Serial.print(temperature, 2);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity, 2);
  Serial.print(" %, LM35 Temp: ");
  Serial.print(probeTemperature, 2);
  Serial.println(" °C");

  LoRa.beginPacket();
  LoRa.print("Counter: ");
  LoRa.print(counter);
  LoRa.print(", Current: ");
  LoRa.print(current, 2);
  LoRa.print("A, Voltage: ");
  LoRa.print(voltage, 2);
  LoRa.print("V, DHT Temp: ");
  LoRa.print(temperature, 2);
  LoRa.print("°C, Hum: ");
  LoRa.print(humidity, 2);
  LoRa.print("%, LM35 Temp: ");
  LoRa.print(probeTemperature, 2);
  LoRa.print("°C");
  LoRa.endPacket();

  counter++;

  delay(2000);  // Delay between packets
}

// Function to Read Current from ACS712
float readACS712(int pin) {
  const int numSamples = 100; // Number of samples for averaging
  long sum = 0;

  for (int i = 0; i < numSamples; i++) {
    int sensorValue = analogRead(pin);
    sum += sensorValue;
    delay(1);
  }

  float avgSensorValue = sum / (float)numSamples;

  // Convert to voltage (5V reference, 10-bit ADC)
  float voltage = (avgSensorValue / 1023.0) * 5.0;

  // Calculate current in amperes
  float current = (voltage - 2.5) / ACS712_SENSITIVITY; // Centered at 2.5V for zero current

  return current;
}

// Function to Read Voltage from Voltage Sensor
float readVoltageSensor(int pin) {
  int sensorValue = analogRead(pin);

  // Convert to voltage (5V reference, 10-bit ADC)
  float voltage = (sensorValue / 1023.0) * 5.0;

  // Scale by the voltage divider ratio
  float actualVoltage = voltage * VOLTAGE_SENSOR_RATIO;

  return actualVoltage;
}

// Function to Read Temperature from LM35
float readLM35(int pin) {
  int sensorValue = analogRead(pin);

  // Convert to voltage (5V reference, 10-bit ADC)
  float voltage = (sensorValue / 1023.0) * 5.0;

  // Calculate temperature in Celsius
  float temperature = voltage / LM35_SCALING_FACTOR;

  return temperature;
}
