#include <LoRa.h> 

const int Signal_LED = 3;   // indicator LED
const int LED = 4;   // relay 1

String inString = "";    // string to hold input
int val = 0;
int SyncWord = 0x22;
 
void setup() {
  Serial.begin(9600);
  pinMode(Signal_LED,OUTPUT);
  pinMode(LED,OUTPUT);
  
  digitalWrite(Signal_LED , LOW); 
  digitalWrite(LED , HIGH);
  
  while (!Serial);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) { // or 915E6
    Serial.println("Starting LoRa failed!");
    while (1);
  }
   LoRa.setSpreadingFactor(12);           // ranges from 6-12,default 7 see API docs
   LoRa.setSignalBandwidth(62.5E3);           // for -139dB (page - 112)
   LoRa.setCodingRate4(8);                   // for -139dB (page - 112)
   LoRa.setSyncWord(SyncWord);           // ranges from 0-0xFF, default 0x12, see API docs
/*
  Serial.print("current spreading factor : ");
  Serial.println(LoRa.getSpreadingFactor());
  Serial.print("current bandwidth : ");
  Serial.println(LoRa.getSignalBandwidth());
  Serial.println("LoRa init succeeded.");
  */
}
bool i=0;
int priviousValue = 0;
int liveValue = 0;

void loop() { 
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) { 
    // read packet    
    while (LoRa.available())
    {
      int inChar = LoRa.read();
      inString += (char)inChar;
      val = inString.toInt();  
      digitalWrite(Signal_LED , HIGH);
      delay(10);
      digitalWrite(Signal_LED , LOW);
      delay(10);     
    }
    inString = "";     
    LoRa.packetRssi();    
  }
  
  Serial.println(val);  
  liveValue = val;
    
  if(priviousValue != liveValue)
  {
    priviousValue = liveValue;
  
    if(val == 11)
    {
      digitalWrite(LED , LOW);
    }
    
    if(val == 22)
    {
      digitalWrite(LED , HIGH);
    }
  
    
  }
  delay(50);
}