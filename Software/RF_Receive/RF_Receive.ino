#include <SPI.h>
#include <RF24.h>

#define CE_PIN  20
#define CSN_PIN 21

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "NODE1";

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("NRF24 NOT DETECTED");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("RECEIVER READY");
}

void loop() {
  if (radio.available()) {
    char text[32] = {0};
    radio.read(&text, sizeof(text));

    Serial.print("Received: ");
    Serial.println(text);
  }
}
