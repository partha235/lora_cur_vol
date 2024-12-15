/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL3Fixv11NV"
#define BLYNK_TEMPLATE_NAME         "iotmonitring"
#define BLYNK_AUTH_TOKEN            "EGdXYvcTNB_UjQ4rNX9t4p98aCTCzyDq"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

// Global variables
double tariffRate = 10; // Tariff rate per kWh
float currentReadings[5] = {0}; // Array to store the last 5 current readings
int currentReadIndex = 0; // Index for the current readings array
double averageAmpsRMS = 0; // Average current in Amps RMS


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Hi";
char pass[] = "123456789";
#define RELAY_PIN_1 2 // Digital pin connected to the first relay module
#define RELAY_PIN_2 4 // Digital pin connected to the second relay module
#define RESET_BUTTON_PIN 32  // GPIO pin connected to the reset button

// OLED display width and height, for a 128x64 display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// OLED reset pin (not used in many cases)
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int sensorIn = 34;
int mVperAmp = 150;  // Calibration for ACS712 5A model
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
#define VOLTAGE_SENSOR_PIN 35

volatile double totalUnitsConsumed = 0.0;
volatile double amountDue = 0.0;
int vol;
#define VOLTAGE_SENSOR_PIN 35
#define NUM_READINGS 5
unsigned long theftStartMillis = 0; // Stores the time when theft condition starts
bool isTheftConditionActive = false; // Indicates if the theft condition is currently met

float voltageReadings[NUM_READINGS]; // Array to store voltage readings
int readIndex = 0;                   // Index for inserting the next reading
float averageVoltage = 0;            // To store the calculated average
float sumVoltage = 0;                // Sum of the current window of readings
void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);  // Initialize the reset button pin as input with pull-up
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);
  analogReadResolution(12); // Set the ADC resolution to 12 bits

for (int i = 0; i < 5; i++) {
    currentReadings[i] = 0;
  }
  // Initialize with the I2C addr 0x3C (for the 128x64)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0,0);      // Start at top-left corner
  for (int i = 0; i < NUM_READINGS; i++) {
    voltageReadings[i] = 0.0; // Initialize all elements to 0.0
  }
}

void loop() {
  Blynk.run();
  Voltage = getVPP();
  VRMS = (Voltage / 2.0) * 0.707;
  AmpsRMS = ((VRMS * 1000) / mVperAmp) - 0.30;
  if (AmpsRMS < 0) AmpsRMS = 0; // Correct negative readings


  // Update current readings and compute average
  currentReadings[currentReadIndex] = AmpsRMS;
  currentReadIndex = (currentReadIndex + 1) % 5;
  averageAmpsRMS = 0;
  for (int i = 0; i < 5; i++) {
    averageAmpsRMS += currentReadings[i];
  }
  averageAmpsRMS /= 5;

  float sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
  float voltage=(sensorValue * (3.3 / 4095.0) * 234.26)/2.2; 
   // Update the rolling sum subtracting the oldest value and adding the new one
  sumVoltage -= voltageReadings[readIndex];
  voltageReadings[readIndex] = voltage;
  sumVoltage += voltageReadings[readIndex];

  // Update read index and wrap around if necessary
  readIndex = (readIndex + 1) % NUM_READINGS;

  // Calculate the average
  averageVoltage = sumVoltage / NUM_READINGS;
  if(averageVoltage>150)
  {
    averageVoltage=averageVoltage;
  }
  else{
    averageVoltage=0;
  }

  // Calculate power and update total energy consumption
  Watt = (averageAmpsRMS * 240 / 1.2);
  double unitsConsumed = (Watt / 1000.0) * (100.0 / 3600.0); // kWh
  totalUnitsConsumed += unitsConsumed;

  // Calculate the amount to be charged based on units consumed
  double amountDue = totalUnitsConsumed * tariffRate;
if (digitalRead(RESET_BUTTON_PIN) == LOW) {  // Check if the button is pressed
    resetAmount();  // Call the function to reset the amount
  }

 if (totalUnitsConsumed >= 0.2 && totalUnitsConsumed <= 0.22)
 {
  Blynk.logEvent("amount");
   display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("  HIGH");
  display.println(" AMOUNT ");
  display.println("  ALERT");
  display.display();
    delay(2000);

}
if (averageAmpsRMS > 0.41) 
{
  Blynk.logEvent("theft");
     display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("  THEFT   ");
  display.println("  ALERT");

  display.display();
  delay(2000);
 }

    delay(100);

  // Blynk app updates
  Blynk.virtualWrite(V2, averageAmpsRMS);
  Blynk.virtualWrite(V3, totalUnitsConsumed);
  Blynk.virtualWrite(V4, amountDue);

  // Serial Monitor Output
  Serial.print("Current (Amps RMS): ");
  Serial.println(averageAmpsRMS);
  Serial.print("Power (Watts): ");
  Serial.println(Watt);
  Serial.print("Total Units (kWh): ");
  Serial.println(totalUnitsConsumed);
  Serial.print("Amount Due ($): ");
  Serial.println(amountDue);

  // OLED Display
  display.clearDisplay();
  display.setTextSize(1); // Increase text size for better readability
  display.setCursor(0,0);
  display.print("Voltage: ");
  display.print(averageVoltage);
    display.println("  vots");
  display.print("current: ");
  display.print(averageAmpsRMS);
    display.println("  amps");
  display.print("power  : ");
  display.print(Watt);
    display.println("  Watts");
  display.print("Units  : ");
  display.print(totalUnitsConsumed, 3);
    display.println("  KWH");
 // Display units with 3 decimal places
  display.print("Rs     : ");
  display.println(amountDue, 2); // Display amount with 2 decimal places
  display.display();

  delay(100);
}

void updateOledDisplay(bool relay1Status, bool relay2Status) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Relay 1: ");
  display.println(relay1Status ? "ON" : "OFF");
  display.print("Relay 2: ");
  display.println(relay2Status ? "ON" : "OFF");
  display.display();
}
BLYNK_WRITE(V0) {
  bool value = param.asInt(); // Get the value from the Blynk app (0 or 1)
  digitalWrite(RELAY_PIN_1, value);
  updateOledDisplay(value, digitalRead(RELAY_PIN_2));
}

BLYNK_WRITE(V1) {
  bool value = param.asInt(); // Get the value from the Blynk app (0 or 1)
  digitalWrite(RELAY_PIN_2, value);
  updateOledDisplay(digitalRead(RELAY_PIN_1), value);
}

float getVPP() {
  float result;
  int readValue;
  int maxValue = 0;
  int minValue = 4096;

  uint32_t start_time = millis();
  while((millis() - start_time) < 1000) {
    readValue = analogRead(sensorIn);
    if(readValue > maxValue) {
      maxValue = readValue;
    }
    if(readValue < minValue) {
      minValue = readValue;
    }
  }
  
  result = ((maxValue - minValue) * 3.3) / 4096.0;
  return result;
}
void resetAmount() {
  totalUnitsConsumed = 0.0;
  amountDue = 0.0;
  Blynk.virtualWrite(V3, totalUnitsConsumed);  // Update Blynk value
  Blynk.virtualWrite(V4, amountDue);  // Update Blynk value

  Serial.println("Amount and units reset to zero.");
}