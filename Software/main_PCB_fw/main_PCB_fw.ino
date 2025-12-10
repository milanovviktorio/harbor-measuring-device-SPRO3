#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <SPI.h>

#include "RF24.h"

const int T_DELAY = 1000;

// BLDC driver defines
#define BEMF 6
#define CLS 20
#define CHS 21
#define BLS 22
#define BHS 23
#define ALS 24
#define AHS 25

// SPI radio defines
#define rCE_PIN  7
#define rCSN_PIN 8
#define rMISO 16
#define rMOSI 19
#define rSCK 18

// SPI radio other shit
RF24 radio(rCE_PIN, rCSN_PIN);

unsigned int motor_step;

void motor_commutate(int step);

void motor_pwm_setup();

void motor_pwm_startup();

void motor_pwm_state(uint pin, uint pin_comp, uint freq, float duty_cycle);

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
  //pinMode(D25,OUTPUT);
  Serial.begin(115200);
  delay(3000);
  //digitalWrite(D25, HIGH);
  motor_pwm_setup();
  //digitalWrite(p25, LOW);
  //radio_setup();
  //digitalWrite(p25, HIGH);
  motor_pwm_startup();
  uint8_t radio_pipe;
}

// the loop function runs over and over again forever
void loop() {
  //if (radio.available(&radio_pipe)) {               // is there a payload? get the pipe number that received it
  //  uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
  //  radio.read(&payload, bytes);            // fetch payload from FIFO
  //}
  
  
}

void motor_commutate(int step) {
    switch(step) {
        case 0:
            motor_pwm_state(ALS, AHS, 20000, 0.8);
            motor_pwm_state(BLS, BHS, 20000, 0.2);
            motor_pwm_state(CLS, CHS, 20000, 0.0);
            break;
        case 1:
            motor_pwm_state(ALS, AHS, 20000, 0.8);
            motor_pwm_state(BLS, BHS, 20000, 0.0);
            motor_pwm_state(CLS, CHS, 20000, 0.2);
            break;
        case 2:
            motor_pwm_state(ALS, AHS, 20000, 0.2);
            motor_pwm_state(BLS, BHS, 20000, 0.0);
            motor_pwm_state(CLS, CHS, 20000, 0.2);
            break;
        case 3:
            motor_pwm_state(ALS, AHS, 20000, 0.0);
            motor_pwm_state(BLS, BHS, 20000, 0.2);
            motor_pwm_state(CLS, CHS, 20000, 0.8);
            break;
        case 4:
            motor_pwm_state(ALS, AHS, 20000, 0.0);
            motor_pwm_state(BLS, BHS, 20000, 0.8);
            motor_pwm_state(CLS, CHS, 20000, 0.2);
            break;
        case 5:
            motor_pwm_state(ALS, AHS, 20000, 0.2);
            motor_pwm_state(BLS, BHS, 20000, 0.8);
            motor_pwm_state(CLS, CHS, 20000, 0.0);
            break;
    }
}

void motor_pwm_setup() {
  gpio_set_function(AHS, GPIO_FUNC_PWM);
  gpio_set_function(ALS, GPIO_FUNC_PWM);
  gpio_set_function(BHS, GPIO_FUNC_PWM);
  gpio_set_function(BLS, GPIO_FUNC_PWM);
  gpio_set_function(CHS, GPIO_FUNC_PWM);
  gpio_set_function(CLS, GPIO_FUNC_PWM);
}

void motor_pwm_startup() {
  motor_pwm_state(ALS, AHS, 20000, 0.8);
  motor_pwm_state(BLS, BHS, 20000, 0.2);
  motor_pwm_state(CLS, CHS, 20000, 0.0);
  motor_step=0;
  delay(200);
  Serial.println("Tried to start motor");
  attachInterrupt(digitalPinToInterrupt(BEMF), BEMF_call, RISING);
  Serial.println("Hand-off to interrupt");
}

void BEMF_call() {
  delayMicroseconds(8);
  motor_step++;
  if(motor_step>5 || motor_step<0)
    motor_step=0;
  motor_commutate(motor_step);
}

void motor_pwm_state(uint pin, uint pin_comp, uint freq, float duty_cycle) {
  uint slice_num = pwm_gpio_to_slice_num(pin);
  uint channel = pwm_gpio_to_channel(pin);
  uint channel2 = pwm_gpio_to_channel(pin_comp);
  
  uint32_t clock = clock_get_hz(clk_sys); // system clock
  uint32_t divider = 1;                   // integer divider
  uint32_t wrap = clock / (divider * freq) - 1;
  
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, wrap);

  pwm_set_output_polarity(slice_num, true, false);

  if (duty_cycle < 0.0f) duty_cycle = 0.0f;
  if (duty_cycle > 1.0f) duty_cycle = 1.0f;

  uint32_t level = (uint32_t)(duty_cycle * wrap);

  pwm_set_chan_level(slice_num, channel, level);
  pwm_set_chan_level(slice_num, channel2, level);

  pwm_set_enabled(slice_num, true);
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


    /*// To set the radioNumber via the Serial terminal on startup
    printf("Which radio is this? Enter '0' or '1'. Defaults to '0'\n");
    char input = getchar();
    radioNumber = input == 49;
    printf("radioNumber = %d\n", (int)radioNumber);
    */

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    /*radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

    // set the TX address of the RX node for use on the TX pipe (pipe 0)
    radio.stopListening(address[radioNumber]);

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

    
    return true;*/
} // setup
