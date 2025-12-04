#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

const int T_DELAY = 1000;

// BLDC driver defines
#define BEMF 6
#define CLS 20
#define CHS 21
#define BLS 22
#define BHS 23
#define ALS 24
#define AHS 25

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

// the setup function runs once when you press reset or power the board
void setup() {
  motor_pwm_setup();
  pinMode(p25, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(p25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(T_DELAY);             // wait for a second
  digitalWrite(p25, LOW);    // turn the LED off by making the voltage LOW
  delay(T_DELAY);             // wait for a second

  
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
  pinMode(AHS, OUTPUT);
  pinMode(ALS, OUTPUT);
  pinMode(BHS, OUTPUT);
  pinMode(BLS, OUTPUT);
  pinMode(CHS, OUTPUT);
  pinMode(CLS, OUTPUT);
}

void motor_pwm_startup() {
  motor_pwm_state(ALS, AHS, 20000, 0.8);
  motor_pwm_state(BLS, BHS, 20000, 0.2);
  motor_pwm_state(CLS, CHS, 20000, 0.0);
  motor_step=0;
  delay(200);

  attachInterrupt(digitalPinToInterrupt(BEMF), BEMF_call, RISING);
}

void BEMF_call() {
  delayMicroseconds(8);
  motor_commutate(motor_step+1);
}

void motor_pwm_state(uint pin, uint pin_comp, uint freq, float duty_cycle) {
  uint slice_num = pwm_gpio_to_slice_num(pin);
  uint channel = pwm_gpio_to_channel(pin);
  
  // Set frequency
  uint32_t clock = 125000000; // Pico default clock
  uint32_t divider = clock / freq / 65536;
  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, 65535);

  pwm_set_chan_level(slice_num, channel, duty_cycle * 65535);
  pwm_set_output_polarity(slice_num, false, true);
  pwm_set_enabled(slice_num, true);
}
