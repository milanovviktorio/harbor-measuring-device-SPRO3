#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <SPI.h>

#include <Wire.h>
#include <U8g2lib.h>

#include "RF24.h"

#define PAYLOAD_SIZE 6

const int T_DELAY = 1000;

bool RF_state=0; //0 for receiving, 1 for transmitting

// SPI radio defines
const int rCE_PIN = 20;
const int rCSN_PIN = 21;
const int rMISO = 16;
const int rMOSI = 19;
const int rSCK = 18;

// SPI radio init
RF24 radio(rCE_PIN, rCSN_PIN);

// Analog stick defines
const int AS0 = 26;
const int AS1 = 27;

// Button defines
const int BPin0 = 4;
const int BPin1 = 5;
const int BPin2 = 6;

// Constructor for I2C, SH1106, 128×64, hardware I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,/* reset=*/ U8X8_PIN_NONE);

uint8_t boat_bat_level;
uint16_t cur_depth;


/* DEVELOPMENT CHECKLIST
    This shit needs to:
    * Receive commands from the RC remote
    * Send info back to the remote
    * Control the main motor and the rudder
    * Controlling the main motor specifically entails strobing the phases
    * Take measurements from the gyro
    * Take measurements of the bottom from the sonar
*/

/*
  RC stack checklist because I can't think for how to do this for shit rn. The stack needs to:
    * Init SPI and start receiving payloads from the remote. How long should the payloads be? 4 bytes maybe?
    * Process those payloads into something more manageable (whether they're control signals for the steering, process those into what the motors should be doing)
    * Say like the controller sends 4 bytes with a status byte, and then a few bytes that more or less send the position of the analog sticks, more or less
    * Probably send a status message every once in a while to make sure the connection is still good (I'll need to check how the example code handles this)
    * The same formula for what the boat needs to send back
    * Massive if statement might be incoming
*/

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr);
  oled.drawFrame(17, 20, 88, 15); //x, y ,dimesnsions x and y
  oled.drawStr(20, 32, "MAZNA BATIO"); //x, y and text
  oled.sendBuffer();
  delay(1000);
  
  radio_setup();
  
  uint8_t rf_packet[6],pipe;
  uint8_t AS0_val, AS1_val;
}

// the loop function runs over and over again forever
void loop() {
  radio.startListening();
  if(radio.available(&pipe)) 
  {
    if(read_frame(rf_packet))
    {
      if(!(process_frame_remote(rf_packet)))
        //make it do something to display an error
    }
  }
  radio.stopListening();
  AS0_val=analogRead(AS0)/4;
  if(AS0_val>249)
    AS0_val=255;
  AS1_val=analogRead(AS1)/4;
  if(AS1_val>249)
    AS1_val=255;
  send_frame(0, AS0_val, AS1_val, 0);
  
}

void radio_setup()
{
    // Let these addresses be used for the pair
    uint8_t address[][6] = {"1Node", "2Node"};
    // It is very helpful to think of an address as a path instead of as
    // an identifying device destination

    // to use different addresses on a pair of radios, we need a variable to
    // uniquely identify which address this radio will use to transmit
    bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

    SPI.setRX(16);   // MISO
    SPI.setTX(19);   // MOSI
    SPI.setSCK(18);  // SCK

    SPI.begin();
    if (!radio.begin()) {
      Serial.println("radio hardware not responding!!");
      while (1) {} // hold program in infinite loop to prevent subsequent errors
    }

    //set radio number. This is the secondary bc it's the remote
    radioNumber=1;

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio.setPayloadSize(PAYLOAD_SIZE); // float datatype occupies 4 bytes

    // set the TX address of the RX node for use on the TX pipe (pipe 0)
    radio.stopListening(address[radioNumber]);

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

    
    return true;*/
} // setup

bool send_frame(uint8_t seq, uint8_t ch1, uint8_t ch2, uint8_t flags) {
  uint8_t pkt[6] = {0xAA, seq, ch1, ch2, flags, 0};
  pkt[5] = crc8(pkt, 5);
  uint64_t start_timer = to_us_since_boot(get_absolute_time());  // start the timer
  bool report = radio.write(&pkt, PAYLOAD_SIZE);             // transmit & save the report
  uint64_t end_timer = to_us_since_boot(get_absolute_time());    // end the timer

  if (report) {
    // payload was delivered; print the payload sent & the timer result
    printf("Transmission successful! Time to transmit = %llu us. Sent: %f\n", end_timer - start_timer, payload);
    return 1;
  } else {
    // payload was not delivered
    return 0;
  }
}

// 0 - signature (always 0xAA)
// 1 - seq (boat battery level) (0xFF for keepalive)
// 2 - pot chan 1 (0x0 for keepalive) (high byte 1 of a uint16_t for depth)
// 3 - pot chan 2 (0x0 for keepalive) (low byte 2 of a uint16_t for depth)
// 4 - flags (0xF for keepalive) (0x1 for boat depth and status)
// 5 - CRC8

bool read_frame(uint8_t* pkt) {
    // Sync on 0xAA, then read 5 more bytes. Implement a ring buffer in practice.
    uint8_t pipe;
    if(radio.available(&pipe)) 
    {
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      if(bytes==6) {
      radio.read(pkt, bytes);                 // fetch payload from FIFO
      if (pkt[0] == 0xAA) {
        uint8_t c = crc8(payload, 5);
        return c == pkt[5];
      }}
    }
    return false;
}

uint8_t process_frame_remote(uint8_t* pkt) {
  uint8_t crc_check = crc8(pkt,5);
  if(pkt[4] == 0)
    {
      boat_bat_level = pkt[1];
      cur_depth = (pkt[2] << 8) | pkt[3];
      return 1;
    }
  else if(pkt[4] == 1)
    {
      if(pkt[1] == 0xFF && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0 && pkt[5] == crc_check)
      return 2;
    }
  return 0;
}

uint8_t crc8(const uint8_t* d, size_t n) {
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++)
            c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1);
    }
    return c;
}

