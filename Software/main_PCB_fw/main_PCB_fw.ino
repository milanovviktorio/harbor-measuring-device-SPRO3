#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include "ICM42688.h"
#include "RF24.h"

const int T_DELAY = 1000;

// BLDC driver defines
const int BEMF = 6;
const int CLS = 20;
const int CHS = 21;
const int BLS = 22;
const int BHS = 23;
const int ALS = 24;
const int AHS = 25;

// SPI radio defines
const int rCE_PIN = 7;
const int rCSN_PIN = 8;
const int rMISO = 16;
const int rMOSI = 19;
const int rSCK = 18;

//Pins for sonar:
const int sonar_MISO = 0;    // SPI MISO
const int sonar_CS  = 1;    // Chip select
const int sonar_SCK  = 2;    // SPI SCK
const int sonar_MOSI = 3;    // SPI MOSI
const int sonar_IO1 = 4;
const int sonar_IO2 = 5;
const int sonar_O3  = 14;
const int sonar_O4  = 15;
const int sonar_analogIn = 26;

//Pins for IMU
const int IMU_CS = 9;
const int IMU_SDA = 10;
const int IMU_SCL = 11;

//Servo pin
const int servo_pin = 17;

//IMU Variables
float cur_gyro_x;
float cur_gyro_y;

//Sonar Variables
int measure_sonarX = 0;
int measure_sonarY = 0;

// Create TwoWire instance for I2C1 using hardware i2c1
TwoWire I2C1_bus(i2c1, IMU_SDA, IMU_SCL);  // i2c1 = hardware I2C1, SDA=10, SCL=11

//Servo init
Servo myservo;

// ICM42688 object using I2C1
ICM42688 IMU(I2C1_bus, 0x68);

// SPI radio init
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
  pinMode(IMU_SDA, INPUT_PULLUP);
  pinMode(IMU_SCL, INPUT_PULLUP);
  pinMode(IMU_CS, INPUT_PULLUP);
  myservo.attach(servo_pin);  // attaches the servo on the appropriate pint o the Servo object
  while (!Serial) {}

  // Initialize I2C1
  I2C1_bus.begin();

  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
  
  Serial.begin(115200);
  delay(3000);
  
  motor_pwm_setup();

  //radio_setup();

  motor_pwm_startup();
  
}

// the loop function runs over and over again forever
void loop() {
  //if (radio.available(&radio_pipe)) {               // is there a payload? get the pipe number that received it
  //  uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
  //  radio.read(&payload, bytes);            // fetch payload from FIFO
  //}
  Serial.println(detect_imu());

  //The most important part of the code!!!
  if(detect_imu() == 1)
  {
    //Get depth from sonar
    //Send the data through RF module to Controller
  }else{
    //The boat moves
    //Get the values from the RF controller for the movement
    //Execute code for the BLDC motor for the boat to move
    //Steer the boat with the servo
  }
  
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

//Everything that the IMU does"
//Gets the values for the Gyro X and Y axis
//Then checks for if the boat is okay to use the sonar
//Return 1 or 0 corresponding to the answer of the previous line
int detect_imu()
{
  IMU.getAGT();
  cur_gyro_x = IMU.accX();
  cur_gyro_y = IMU.accY();
  delay(500);
  if(cur_gyro_x > 0.5)
  {
    Serial.println("X: Tilted right");
    measure_sonarX = 0;
  }else if(cur_gyro_x < -0.5)
  {
    Serial.println("X: Tilted left");
    measure_sonarX = 0;
  }else{
    Serial.println("X: Resting");
    measure_sonarX = 1;
  }
  if(cur_gyro_y > 0.5)
  {
    Serial.println("Y: Tilted forward");
    measure_sonarY = 0;
  }else if(cur_gyro_y < -0.5)
  {
    Serial.println("Y: Tilted backwards");
    measure_sonarY = 0;
  }else{
    Serial.println("Y: Resting");
    measure_sonarY = 1;
  }

  if((measure_sonarX & measure_sonarY) == 1)
  {
    
    Serial.println("Free to measure the sonar");
    return 1;
  }else{
    
    Serial.println("Do not measure the sonar");
    return 0;
  }

  Serial.print(IMU.accX(), 6); Serial.print("\t");
  Serial.print(IMU.accY(), 6); Serial.print("\t");
  //Serial.print(IMU.accZ(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrX(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrY(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrZ(), 6); Serial.print("\t");
  //Serial.println(IMU.temp(), 6);
  Serial.println();
}

