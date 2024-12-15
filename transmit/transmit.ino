/*************************************************************
Transmitter with ACS712, Voltage Sensor, DHT11, and DS18B20
**************************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin Definitions
#define CURRENT_SENSOR_PIN A0  // ACS712 connected to Analog Pin A0
#define VOLTAGE_SENSOR_PIN A1  // Voltage sensor connected to Analog Pin A1
#define DHT_PIN 5        // DHT11 data pin connected to Digital Pin 2
#define DHT_TYPE DHT11        // Specify DHT sensor type
#define ONE_WIRE_BUS 7       // DS18B20 data pin connected to Digital Pin 3

// Constants for Calibration
const float ACS712_SENSITIVITY = 0.185;  // Sensitivity for ACS712-05B (185mV/A)
const float VOLTAGE_SENSOR_RATIO = 11.0; // Voltage divider ratio (e.g., 100k:10k)

float multiplier =0.185; //Sensibility in Voltios/Ampers for the 5A model

// LEDs for mode indication
int led1 = A2;
int led2 = A3;

// Variables
int counter = 0;  // Packet counter
float current = 0.0;
float voltage = 0.0;
float temperature = 0.0;       // DHT11 Temperature
float humidity = 0.0;          // DHT11 Humidity
float dsTemperature = 0.0;     // DS18B20 Temperature

// DHT Sensor Object
DHT dht(DHT_PIN, DHT_TYPE);

// OneWire and DallasTemperature objects
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Serial.begin(115200);
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

  // Initialize DS18B20 sensor
  sensors.begin();
}

void loop() {
  // Read Current from ACS712
  current = readACS712(CURRENT_SENSOR_PIN);

  // Read Voltage from Voltage Sensor
  voltage = readVoltageSensor(VOLTAGE_SENSOR_PIN);

  // Read Temperature and Humidity from DHT11
  temperature = dht.readTemperature();  // Read temperature in Celsius
  humidity = dht.readHumidity();        // Read relative humidity

  // Read Temperature from DS18B20
  sensors.requestTemperatures(); // Request temperature from DS18B20
  dsTemperature = sensors.getTempCByIndex(0);  // Get temperature from first DS18B20 sensor

  // Check if DHT11 readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 0.0;
    humidity = 0.0;
  }

  // Send Data via LoRa
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
  Serial.print(" %, DS18B20 Temp: ");
  Serial.print(dsTemperature, 2);
  Serial.println(" °C");

  LoRa.beginPacket();
  LoRa.print("Counter: ");
  LoRa.print(counter);
  LoRa.print(", Current: ");
  LoRa.print(current, 2);
  LoRa.print("A, Voltage: ");
  LoRa.print(voltage, 2);
  // LoRa.print("V, DHT Temp: ");
  // LoRa.print(temperature, 2);
  LoRa.print(" °C, Hum: ");
  LoRa.print(humidity, 2);
  LoRa.print("%, DS18B20 Temp: ");
  LoRa.print(dsTemperature, 2);
  LoRa.print(" °C");
  LoRa.endPacket();

  counter++;

  delay(2000);  // Delay between packets
}

// Function to Read Current from ACS712
float readACS712(int pin) {

    int sensorValue = analogRead(pin);

  float SensorRead = sensorValue*(5.0 / 1023.0);     //We read the sensor output  
  float Current = abs(((SensorRead-2.5)/multiplier)*2);                  //Calculate the current value

  return Current;
}

// Function to Read Voltage from Voltage Sensor
float readVoltageSensor(int pin) {
  int sensorValue = analogRead(pin);

  // Convert to voltage (5V reference, 10-bit ADC)
  float voltage =( (sensorValue / 1023.0) * 5.0)*6.3;
  if(voltage<=10){
    voltage=0;
  }

  // Scale by the voltage divider ratio
  float actualVoltage = voltage * VOLTAGE_SENSOR_RATIO;

  return actualVoltage;
}
