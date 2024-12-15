/******************************************************************
this program for transmitter and also receive data when switch on.
********************************************************************/

#include <SPI.h>
#include <LoRa.h>

// Pin Definitions
#define LORA_FREQUENCY 433E6
#define TRANSMIT_PIN 3 // Pin to trigger transmission

int counter = 0;         // Transmission counter
int val = 0;             // Parsed integer value from received data
int trsState = 1;        // State of the transmit pin
String inString = "";    // Buffer for received data

void setup() {
  // Initialize Transmit Pin
  pinMode(TRANSMIT_PIN, INPUT);

  // Initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender/Receiver");

  // Initialize LoRa
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1); // Halt on failure
  }

  LoRa.setTxPower(20);
  Serial.println("LoRa Initialized");
}

void loop() {
  // Read the state of the transmit pin
  trsState = digitalRead(TRANSMIT_PIN);
  Serial.println(trsState);
  if (trsState) { 
    // Send Data
    Serial.print("Sending packet: ");
    Serial.println(counter);

    LoRa.beginPacket();
    LoRa.print("1"); // Send a "1" as the sensor data
    LoRa.endPacket();

    counter++;
    delay(500); // Short delay after sending
  } else {
    // Receive Data
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.print("Received packet: '");

      while (LoRa.available()) {
        char inChar = (char)LoRa.read();
        Serial.print(inChar); // Print received character
        inString += inChar;   // Append character to the string
      }

      Serial.println("'"); // End of received packet

      // Convert received string to integer
      val = inString.toInt();
      Serial.print("Parsed Integer Value: ");
      Serial.println(val);

      // Clear the string for the next packet
      inString = "";
    }
  }

  delay(500); // Short delay before the next operation
}
