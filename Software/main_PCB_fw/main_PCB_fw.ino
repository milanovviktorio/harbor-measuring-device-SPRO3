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

#define PAYLOAD_SIZE 6

//timer shit

const float DRIVE_FREQUENCY = 40000.0f;  // 40 kHz example
const uint BURST_PERIOD_US = 1e6 / (DRIVE_FREQUENCY * 2);  // Half-cycle

// ---------------------- DRIVE FREQUENCY SETTINGS ----------------------
// Sets the output frequency of the ultrasonic transducer
// Uses DRIVE_FREQUENCY directly for R4, uses divider for R3
#define DRIVE_FREQUENCY 40000

// ---------------------- BANDPASS FILTER SETTINGS ----------------------
// Sets the digital band-pass filter frequency on the TUSS4470 driver chip
// This should roughly match the transducer drive frequency
// For additional register values, see TUSS4470 datasheet, Table 7.1 (pages 17–18)
#define FILTER_FREQUENCY_REGISTER 0x00 // 40 kHz
// #define FILTER_FREQUENCY_REGISTER 0x09 // 68 kHz
// #define FILTER_FREQUENCY_REGISTER 0x10 // 100 kHz
// #define FILTER_FREQUENCY_REGISTER 0x18 // 151 kHz
// #define FILTER_FREQUENCY_REGISTER 0x1E // 200 kHz

// Number of ADC samples to take per measurement cycle
// Each sample takes approximately 13.2 microseconds
// This value must match the number of samples expected by the Python visualization tool
// Max 1800 on R3, ~10000 on R4
#define NUM_SAMPLES 1800

// Number of initial samples to ignore after sending the transducer pulse
// These ignored samples represent the "blind zone" where the transducer is still ringing
#define BLINDZONE_SAMPLE_END 450

// Threshold level for detecting the bottom echo
// The first echo stronger than this value (after the blind zone) is considered the bottom
#define THRESHOLD_VALUE 0x19

struct __attribute__((packed)) Frame { // the TUSS code uses frame structs for storing depth data. This may need to be taken out later
  uint8_t  start = 0xAA;
  uint16_t  depth_index;            
  int16_t  temp_scaled;     
  uint16_t vDrv_scaled;     
  uint8_t  samples[NUM_SAMPLES];
  uint8_t  checksum;         
};
static Frame frame;

volatile bool burstFlag = false;

// Create TwoWire instance for I2C1 using hardware i2c1
TwoWire I2C1_bus(i2c1, IMU_SDA, IMU_SCL);  // i2c1 = hardware I2C1, SDA=10, SCL=11

//Servo init
Servo myservo;

// ICM42688 object using I2C1
ICM42688 IMU(I2C1_bus, 0x68);

// SPI radio init
RF24 radio(rCE_PIN, rCSN_PIN);

//Sonar stuff
SPIClassRP2040 sonarSPI(spi0, sonar_MISO, sonar_CS, sonar_SCK, sonar_MOSI);

bool read_frame(uint8_t* pkt);
bool send_frame(uint8_t seq, uint8_t ch1, uint8_t ch2, uint8_t flags);
uint8_t crc8(const uint8_t* d, size_t n);
uint8_t process_frame_remote(uint8_t* pkt);

uint8_t AS0_val=0, AS1_val=0;

unsigned int motor_step;

void motor_commutate(int step);

void motor_pwm_setup();

void motor_pwm_startup();

void motor_pwm_state(uint pin, uint pin_comp, uint freq, float duty_cycle);

/*current dev checklist:
  * Lots of testing needs to be done. Rn everything is building blocks. And the main function is kinda empty.
  * RC needs testing. I think it should be functional, but I haven't been able to test it yet.
  * The ESC code should be functional, it can be tested, but I haven't fully thought of a way to handle starting and stopping the motor. 
  Probably a simple function that deactivates the interrupt and then sets all duty cycles to 0
  * The TUSS code is very very questionable. I used a lot of Copilot to port the basics of the openecho code for it, if everything goes well it should be able to initiate the TUSS, 
  get a ToF reading, and do some math to turn that into depth
  * The IMU just works

  The big challenge: Program flow. Go ask ChatGPT about it I guess. Lots of tape required
*/

/* old DEVELOPMENT CHECKLIST
    This shit needs to:
    * Receive commands from the RC remote
    * Send info back to the remote
    * Control the main motor and the rudder
    * Controlling the main motor specifically entails strobing the phases
    * Take measurements from the gyro
    * Take measurements of the bottom from the sonar
*/

/*
  old RC stack checklist because I can't think for how to do this for shit rn. The stack needs to:
    * Init SPI and start receiving payloads from the remote. How long should the payloads be? 4 bytes maybe?
    * Process those payloads into something more manageable (whether they're control signals for the steering, process those into what the motors should be doing)
    * Say like the controller sends 4 bytes with a status byte, and then a few bytes that more or less send the position of the analog sticks, more or less
    * Probably send a status message every once in a while to make sure the connection is still good (I'll need to check how the example code handles this)
    * The same formula for what the boat needs to send back
    * Massive if statement might be incoming
*/

byte misoBuf[2];  // SPI receive buffer
byte inByteArr[2];  // SPI transmit buffer

volatile int pulseCount = 0;
volatile int sampleIndex = 0;

volatile bool detectedDepth = false;  // Condition flag
volatile uint16_t depthDetectSample = 0;

uint16_t raw12;
uint8_t v;

// Callback function (like burstCallback). Strobes IO2 while the ADC is trying to do stuff
void burstCallback(unsigned int alarm_num) {
  digitalWrite(sonar_IO2, !digitalRead(sonar_IO2));
  pulseCount++;
  if (pulseCount >= 32) {
    burstFlag=true;
    pulseCount = 0;  // Reset counter for next cycle
  }

  // Reschedule for periodic behavior
  hardware_alarm_set_target(0, timer_hw->timerawl + BURST_PERIOD_US);
}

void setupBurstTimer() {
  hardware_alarm_set_callback(0, burstCallback);
  hardware_alarm_set_target(0, timer_hw->timerawl + BURST_PERIOD_US);
}

void burstTimerStart() {
    burstFlag = false;
    pulseCount = 0;
    digitalWrite(sonar_IO2, LOW);
    hardware_alarm_set_callback(0, burstCallback);
    hardware_alarm_set_target(0, timer_hw->timerawl + BURST_PERIOD_US);
}

byte tuss4470Read(byte addr) {
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);  // Set read bit and address
  inByteArr[1] = 0x00;  // Empty data byte
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));

  return misoBuf[1];
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;  // Set write bit and address
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}

byte tuss4470Parity(byte* spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

void spiTransfer(byte* mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));

  digitalWrite(sonar_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = sonarSPI.transfer(mosi[i]);
  }
  digitalWrite(sonar_CS, HIGH);
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;  // Combine high and low bytes
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;  // Odd parity calculation
}

void handleInterrupt() {
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(IMU_SDA, INPUT_PULLUP);
  pinMode(IMU_SCL, INPUT_PULLUP);
  pinMode(IMU_CS, INPUT_PULLUP);
  myservo.attach(servo_pin);  // attaches the servo on the appropriate pint o the Servo object
  while (!Serial) {}

  // Initialize I2C1
  I2C1_bus.begin();

  //Sonar shit
  sonarSPI.begin();
  sonarSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 

  pinMode(sonar_CS, OUTPUT);
  digitalWrite(sonar_CS, HIGH);

  pinMode(sonar_IO1, OUTPUT);
  digitalWrite(sonar_IO1, HIGH);
  pinMode(sonar_IO2, OUTPUT);
  pinMode(sonar_O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sonar_O4), handleInterrupt, RISING);

  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);  // Set BPF center frequency
  tuss4470Write(0x16, 0xF);  // Enable VDRV (not Hi-Z)
  tuss4470Write(0x1A, 0x0F);  // Set burst pulses to 16
  tuss4470Write(0x17, THRESHOLD_VALUE); // enable threshold detection on OUT_4

  setupBurstTimer(); 

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

  radio_setup();

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
    // Trigger time-of-flight measurement
    tuss4470Write(0x1B, 0x01);
    
    burstTimerStart(); //starts the burst timer if it's not already running. I think. I dunno. Copilot seems confused
    //The burst timer's job is to strobe IO2 while the 
    if (burstFlag) {
        burstFlag = false;
        // Your burst logic here
        Serial.println("Burst triggered");

    for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {
        // Start conversion and wait until done
             // blocking, returns 12-bit 0–4095
        analogRead(raw12);
        // Match your original: 12-bit >> 4 -> 8-bit
        v = raw12 >> 4;
        frame.samples[sampleIndex] = v;

        delayMicroseconds(11);           // or adjust to match your acquisition timing

        if (sampleIndex == BLINDZONE_SAMPLE_END) {
            detectedDepth = false;
        }
    }}

    tuss4470Write(0x1B, 0x00);

    float time_of_flight = depthDetectSample * 13.2e-6f;
    float depth_m = (time_of_flight * 1450.0f) / 2.0f;

    Serial.print("depth or something is");
    Serial.println(depth_m);
  }
  else
  {
    //The boat moves
    //Get the values from the RF controller for the movement
    //Execute code for the BLDC motor for the boat to move
    //Steer the boat with the servo
  }
  
}

void motor_commutate(int step) {
    switch(step) {
        case 0:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.8);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.2);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.0);
            break;
        case 1:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.8);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.0);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.2);
            break;
        case 2:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.2);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.0);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.2);
            break;
        case 3:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.0);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.2);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.8);
            break;
        case 4:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.0);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.8);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.2);
            break;
        case 5:
            motor_pwm_state(ALS, AHS, 20000, AS1_val*0.2);
            motor_pwm_state(BLS, BHS, 20000, AS1_val*0.8);
            motor_pwm_state(CLS, CHS, 20000, AS1_val*0.0);
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

} // setup

bool send_frame(uint8_t seq, uint16_t depth, uint8_t flags) {
  uint8_t low,high;
  high = (uint8_t)(depth >> 8);   // upper 8 bits
  low  = (uint8_t)(depth & 0xFF); // lower 8 bits
  uint8_t pkt[6] = {0xAA, seq, high, low, flags, 0};
  pkt[5] = crc8(pkt, 5);
  uint64_t start_timer = to_us_since_boot(get_absolute_time());  // start the timer
  bool report = radio.write(&pkt, PAYLOAD_SIZE);             // transmit & save the report
  uint64_t end_timer = to_us_since_boot(get_absolute_time());    // end the timer

  if (report) {
    // payload was delivered; print the payload sent & the timer result
    printf("Transmission successful! Time to transmit = %llu us. Sent: %f\n", end_timer - start_timer, pkt);
    return 1;
  } else {
    // payload was not delivered
    return 0; 
  }
}

bool send_frame_keepalive() {
  uint8_t pkt[6] = {0xAA, 0xFF, 0, 0, 1, 0};
  pkt[5] = crc8(pkt, 5);
  uint64_t start_timer = to_us_since_boot(get_absolute_time());  // start the timer
  bool report = radio.write(&pkt, PAYLOAD_SIZE);             // transmit & save the report
  uint64_t end_timer = to_us_since_boot(get_absolute_time());    // end the timer

  if (report) {
    return 1;
  } else {
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
        uint8_t c = crc8(pkt, 5);
        return c == pkt[5];
      }}
    }
    return false;
}

uint8_t process_frame_main(uint8_t* pkt) {
  uint8_t crc_check = crc8(pkt,5);
  if(pkt[4] == 0)
    {
      AS0_val = pkt[2];
      AS1_val = pkt[3];
      return 1;
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