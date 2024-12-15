/***************************************************************
this program for receiver and also transmit data when switch on.
***************************************************************/
#include <SPI.h>
#include <LoRa.h>

// Pin Definitions
#define BUTTON_PIN 3       // Button to trigger transmission
#define TRS_PIN 4          // Indicator pin for transmission/reception
#define LED_PIN 13         // LED to indicate received "1"

// Variables
int counter = 0;           // Packet counter for transmission
bool ledState = false;     // Current state of the LED
String inString = "";      // Buffer for received data

void setup() {
  // Initialize Pins
  pinMode(BUTTON_PIN, INPUT);
  pinMode(TRS_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(TRS_PIN, LOW);  // Set transmission indicator LOW
  digitalWrite(LED_PIN, LOW);  // Turn off LED initially

  // Initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender/Receiver");

  // Initialize LoRa
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);  // Halt on failure
  }

  LoRa.setTxPower(20);  // Set transmission power
  Serial.println("LoRa Initialized");
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH) {
    // Transmission Mode
    digitalWrite(TRS_PIN, HIGH); // Indicate transmission mode
    Serial.print("Sending packet: ");
    Serial.println(counter);

    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter); // Include counter in the message
    LoRa.endPacket();

    counter++;
    delay(500);  // Delay after sending
  } else {
    Serial.println("waiting");
    // Reception Mode
    digitalWrite(TRS_PIN, LOW); // Indicate reception mode

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.print("Received packet: '");
      while (LoRa.available()) {
        char inChar = (char)LoRa.read();
        Serial.print(inChar);  // Print received character
        inString += inChar;    // Append character to the string
      }
      Serial.println("'"); // End of received packet

      // Process Received Data
      inString.trim(); // Remove extra spaces or newlines
      if (inString == "1") {
        ledState = !ledState;              // Toggle LED state
        digitalWrite(LED_PIN, ledState);  // Update LED state
        Serial.print("LED State: ");
        Serial.println(ledState ? "ON" : "OFF");
        delay(5000); // Delay to stabilize toggling
      }

      // Clear the string for the next packet
      inString = "";
    }
  }

  delay(500);  // Short delay before the next operation
}
