```c++
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
```