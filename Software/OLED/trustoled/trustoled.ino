#include <Wire.h>
#include <U8g2lib.h>

// Constructor for I2C, SH1106, 128×64, hardware I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  oled.begin();
}

void loop() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr);
  oled.drawFrame(17, 20, 88, 15); //x, y ,dimesnsions x and y
  oled.drawStr(20, 32, "MAZNA BATIO"); //x, y and text
  oled.sendBuffer();
  delay(1000);
}
