#include "ICM42688.h"
#include <Wire.h>

const int CS = 9;
const int SDA_PIN = 10;
const int SCL_PIN = 11;

float init_gyro_x = 0;
float init_gyro_y = 0;
float cur_gyro_x;
float cur_gyro_y;

int measure_sonarX = 0;
int measure_sonarY = 0;

// Create TwoWire instance for I2C1 using hardware i2c1
TwoWire I2C1_bus(i2c1, SDA_PIN, SCL_PIN);  // i2c1 = hardware I2C1, SDA=10, SCL=11

// ICM42688 object using I2C1
ICM42688 IMU(I2C1_bus, 0x68);

void setup() {
  Serial.begin(9600);  
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  pinMode(CS, INPUT_PULLUP);
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

void loop() {
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
  }else{
    Serial.println("Do not measure the sonar");
  }

  Serial.print(IMU.accX(), 6); Serial.print("\t");
  Serial.print(IMU.accY(), 6); Serial.print("\t");
  //Serial.print(IMU.accZ(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrX(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrY(), 6); Serial.print("\t");
  //Serial.print(IMU.gyrZ(), 6); Serial.print("\t");
  //Serial.println(IMU.temp(), 6);
  Serial.println();
  //delay(1000);
}
