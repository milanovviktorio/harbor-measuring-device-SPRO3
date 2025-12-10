//Libraries used
#include "ICM42688.h"
#include <Wire.h>
#include <Servo.h>

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

//Setting up the pins and protocols
void setup() {
  Serial.begin(9600);  
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
}

//Loop
void loop()
{
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
