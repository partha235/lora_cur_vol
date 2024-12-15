#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Settings
#define SCREEN_WIDTH 128              // OLED display width, in pixels
#define SCREEN_HEIGHT 64              // OLED display height, in pixels
#define OLED_RESET    -1              // Reset pin (or -1 if not used)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  // Initialize LoRa
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("waiting for data from lora-1");
  display.display();
  delay(2000);  // Wait for the OLED to display the message
}

void loop() {
  // Try to parse the incoming packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // Received a packet
    Serial.print("Received packet '");

    // Read and display packet content
    String packetContent = "";
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      packetContent += c;
      Serial.print(c);  // Print to Serial Monitor
    }

    // Print RSSI of the packet
    Serial.print("' with RSSI ");
    int packetRssi = LoRa.packetRssi();
    Serial.println(packetRssi);

    // Display packet information on the OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Lora-1 Values");
    if (packetContent.length() > 14) {
      display.setCursor(0, 10);
      display.print(packetContent.substring(14, 28));  // First 16 characters
      display.setCursor(0, 20);
      display.print(packetContent.substring(30, 45));  // Next 16 characters
      display.setCursor(0, 30);
      display.print(packetContent.substring(51, 62));  // Next 16 characters
      display.setCursor(0, 40);
      display.print(packetContent.substring(72, 88));
    } else {
      display.print(packetContent);  // If it's short, just print the full content
    }

    // Display RSSI
    

    // display.print(packetRssi);  // Display RSSI value
    display.display();
  }
}
