// UART2 on ESP32: RX = GPIO16, TX = GPIO17
HardwareSerial USSerial(2);

void setup() {
  Serial.begin(9600);
  USSerial.begin(9600, SERIAL_8N1, 16, 17);  // baud, mode, RX, TX

  Serial.println("Ultrasonic UART Test Starting...");
}

void loop() {
  // Request a reading (US-100 requires sending 0x55)
  USSerial.write(0x55);
  delay(50);

  // Wait for 2-byte response
  if (USSerial.available() >= 2) {
    byte highByte = USSerial.read();
    byte lowByte = USSerial.read();

    int distance = (highByte << 8) | lowByte;  // combine bytes

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");   // US-100 returns mm
  }

  delay(200);
}
