/*************************************************************
Transmitter with ACS712, Voltage Sensor, DHT11, and DS18B20
**************************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Settings
#define SCREEN_WIDTH 128 // OLED width
#define SCREEN_HEIGHT 64 // OLED height
#define OLED_RESET -1    // Reset pin (use -1 if using default reset)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Definitions
#define CURRENT_SENSOR_PIN A0  // ACS712 connected to Analog Pin A0
#define VOLTAGE_SENSOR_PIN A1  // Voltage sensor connected to Analog Pin A1
#define DHT_PIN 2             // DHT11 data pin connected to Digital Pin 2
#define DHT_TYPE DHT11        // Specify DHT sensor type
#define ONE_WIRE_BUS 3       // DS18B20 data pin connected to Digital Pin 3


// Constants for Calibration
const float ACS712_SENSITIVITY = 0.185;  // Sensitivity for ACS712-05B (185mV/A)
const float VOLTAGE_SENSOR_RATIO = 11.0; // Voltage divider ratio (e.g., 100k:10k)

//  leds for mode indication
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
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
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

  // Initialize DS18B20 sensor
  sensors.begin();

  // Initialize OLED

 // Initialize with the I2C addr 0x3C (for the 128x64)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);  // Don't proceed, loop forever
  }
  
  display.display();  // Initialize with the buffer content
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
  LoRa.print("V, DHT Temp: ");
  LoRa.print(temperature, 2);
  LoRa.print("°C, Hum: ");
  LoRa.print(humidity, 2);
  LoRa.print("%, DS18B20 Temp: ");
  LoRa.print(dsTemperature, 2);
  LoRa.print("°C");
  LoRa.endPacket();

  counter++;

  // Update OLED Display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("LoRa Transmitter");
  display.print("Packet: ");
  display.println(counter);
  display.print("Current: ");
  display.print(current, 2);
  display.println(" A");
  display.print("Voltage: ");
  display.print(voltage, 2);
  display.println(" V");
  display.print("DHT Temp: ");
  display.print(temperature, 2);
  display.println(" C");
  display.print("Humidity: ");
  display.print(humidity, 2);
  display.println(" %");
  display.print("DS18B20 Temp: ");
  display.print(dsTemperature, 2);
  display.println(" C");
  display.display();

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
