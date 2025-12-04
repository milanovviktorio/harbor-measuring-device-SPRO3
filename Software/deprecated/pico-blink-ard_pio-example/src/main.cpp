#include <Arduino.h>

const int T_DELAY = 1000;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PC13 as an output.
  pinMode(p25, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(p25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(T_DELAY);             // wait for a second
  digitalWrite(p25, LOW);    // turn the LED off by making the voltage LOW
  delay(T_DELAY);             // wait for a second
}
