/***********************************
transmitter with sensor
**************************/

#include <SPI.h>
#include <LoRa.h>

// Pin Definitions
#define CURRENT_SENSOR_PIN A0  // ACS712 connected to Analog Pin A0
#define VOLTAGE_SENSOR_PIN A1  // Voltage sensor connected to Analog Pin A1

// Constants for Calibration
const float ACS712_SENSITIVITY = 0.185;  // Sensitivity for ACS712-05B (185mV/A)
const float VOLTAGE_SENSOR_RATIO = 11.0; // Voltage divider ratio (e.g., 100k:10k)

// Variables
int counter = 0;  // Packet counter
float current = 0.0;
float voltage = 0.0;

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
}

void loop() {
  // Read Current from ACS712
  current = readACS712(CURRENT_SENSOR_PIN);

  // Read Voltage from Voltage Sensor
  voltage = readVoltageSensor(VOLTAGE_SENSOR_PIN);

  // Send Data
  Serial.print("Sending packet: ");
  Serial.println(counter);
  Serial.print("current");
  Serial.println(current);
  LoRa.beginPacket();
  LoRa.print("Counter: ");
  LoRa.print(counter);
  LoRa.print(", Current: ");
  LoRa.print(current, 2);  // Two decimal places
  LoRa.print("A, Voltage: ");
  LoRa.print(voltage, 2);  // Two decimal places
  LoRa.print("V");
  LoRa.endPacket();

  counter++;

  delay(1000);  // Delay between packets
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
