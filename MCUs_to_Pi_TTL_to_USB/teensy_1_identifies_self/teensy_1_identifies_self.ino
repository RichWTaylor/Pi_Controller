// Teensy 1 - Auto-identification and data transmission
// Save as teensy1.ino

bool setupComplete = false;

void setup() {
  Serial.begin(9600);   // Debug via USB
  Serial1.begin(9600);  // TTL UART communication
  delay(1000);

  // Send identification until acknowledged
  while (!setupComplete) {
    // Send identifier
    Serial.println("Sending identifier...");
    Serial1.println("TEENSY1_ID");

    // Wait for acknowledgment
    delay(500);
    if (Serial1.available()) {
      String response = Serial1.readStringUntil('\n');
      Serial.print("Received: ");
      Serial.println(response);

      if (response == "ACK") {
        Serial.println("!!!!! Setup complete, proceeding to main loop");
        setupComplete = true;
      }
    }
  }
}

void loop() {
  // Regular data transmission
  float float_data = 123.45;  // Replace with your actual sensor data

  // Create data packet
  uint8_t data_packet[7];
  data_packet[0] = 0x3C;  // Start marker '<'

  // Copy float bytes directly (little-endian)
  memcpy(&data_packet[1], &float_data, 4);

  // Calculate checksum
  data_packet[5] = 0;
  for (int i = 1; i < 5; ++i) {
    data_packet[5] ^= data_packet[i];
  }

  data_packet[6] = 0x3E;  // End marker '>'

  // Send the packet
  Serial1.write(data_packet, sizeof(data_packet));

  delay(1000);  // Send once per second
}
