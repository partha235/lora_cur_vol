// LORA code for Transmitting Side

#include <SPI.h>

#include <LoRa.h>

int counter = 0;

void setup() {

  Serial.begin(9600);

  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {

    Serial.println("Starting LoRa failed!");

    while (1);

  }

  LoRa.setTxPower(20);
}

void loop() {

  Serial.print("Sending packet: ");

  Serial.println(counter);

  // send packet

  LoRa.beginPacket();

  LoRa.print(" usb0");

  // LoRa.print(counter);

  LoRa.endPacket();

  counter++;

  delay(100);

  // try to parse packet

  int packetSize = LoRa.parsePacket();

  if (packetSize) {

    // received a packet

    Serial.print("Received packet '");

    // read packet

    while (LoRa.available()) {

      Serial.print((char)LoRa.read());

    }

    // print RSSI of packet

    Serial.print("' with RSSI ");

    Serial.println(LoRa.packetRssi());

  }
  delay(100);

}