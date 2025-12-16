#include <SPI.h>
#include <RF24.h>

#define CE_PIN  20
#define CSN_PIN 21

RF24 radio(CE_PIN, CSN_PIN);

// Address must be same on RX
const byte address[6] = "NODE1";

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("NRF24 NOT DETECTED");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);     // LOW for close testing
  radio.setDataRate(RF24_1MBPS);     // Most stable
  radio.setChannel(108);             // Avoid WiFi
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("TRANSMITTER READY");
}

void loop() {
  const char text[] = "BOMBOCLAT";
  bool ok = radio.write(&text, sizeof(text));

  Serial.print("Send: ");
  Serial.println(ok ? "OK" : "FAIL");

  delay(1000);
}