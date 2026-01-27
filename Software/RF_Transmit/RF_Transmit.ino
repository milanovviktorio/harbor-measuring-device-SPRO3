#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <SPI.h>

#include <Wire.h>
#include <U8g2lib.h>

#include "RF24.h"

const int CE_PIN = 20;
const int CSN_PIN = 21;

const int rMISO = 16;
const int rMOSI = 19;
const int rSCK = 18;

// Analog stick defines
const int AS0 = 26;
const int AS1 = 27;

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,/* reset=*/ U8X8_PIN_NONE);

uint8_t AS0_val, AS1_val;

RF24 radio(CE_PIN, CSN_PIN);

// Address must be same on RX
const byte address[6] = "NODE1";

void setup() {
  Serial.begin(115200);
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr);
  //oled.drawFrame(17, 20, 88, 15); //x, y ,dimesnsions x and y
  //oled.drawStr(20, 32, "MAZNA BATIO"); //x, y and text
  //oled.sendBuffer();
  delay(1000);

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
  
  oled.clearBuffer();
  AS0_val=analogRead(AS0)/4;
  if(AS0_val<25)
    AS0_val=0;
  if(AS0_val>249)
    AS0_val=255;
  AS1_val=analogRead(AS1)/4;
  if(AS1_val<25)
    AS1_val=0;
  if(AS1_val>249)
    AS1_val=255;
  send_frame(0, AS0_val, AS1_val, 0);
  oled.setCursor(0,0);
  oled.print(AS0_val);
  oled.setCursor(0,20);
  oled.print(AS1_val);
  oled.setCursor(0,40);
  oled.print(cur_depth);

  delay(1000);
}