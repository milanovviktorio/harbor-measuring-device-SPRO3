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

// Constructor for I2C, SH1106, 128×64, hardware I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0,/* reset=*/ U8X8_PIN_NONE);

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
  
}

// the loop function runs over and over again forever
void loop() {
  //if (radio.available(&radio_pipe)) {               // is there a payload? get the pipe number that received it
  //  uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
  //  radio.read(&payload, bytes);            // fetch payload from FIFO
  //}
  
  
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

void radio_comm() {
  if (RF_state) {
  // This device is a TX node

  uint64_t start_timer = to_us_since_boot(get_absolute_time());  // start the timer
  bool report = radio.write(&payload, PAYLOAD_SIZE);             // transmit & save the report
  uint64_t end_timer = to_us_since_boot(get_absolute_time());    // end the timer

  if (report) {
    // payload was delivered; print the payload sent & the timer result
    printf("Transmission successful! Time to transmit = %llu us. Sent: %f\n", end_timer - start_timer, payload);
  } else {
    // payload was not delivered
    printf("Transmission failed or timed out\n");
  }

    // to make this example readable in the serial terminal
    sleep_ms(1000); // slow transmissions down by 1 second
  }
  else {
      // This device is a RX node
      uint8_t pipe;
      if (radio.available(&pipe)) {               // is there a payload? get the pipe number that received it
          uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
          radio.read(&payload, bytes);            // fetch payload from FIFO
      }
  } // role
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

