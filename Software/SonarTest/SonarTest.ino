#define TUSS_CLK_PIN 0
#define PING_LENGTH_US 50   // number of cycles at 200 kHz (≈250us)
#define ECHO_ADC_PIN A0     // TUSS4470 echo envelope

int MAX_TIMEOUT = 2000; //for 2m
int THRESHOLD = 250;

void setup() {
  Serial.begin(115200);
  pinMode(TUSS_CLK_PIN, OUTPUT);
  // 200 kHz PWM: period = 5 microseconds
  analogWriteFreq(200000); // some Pico cores allow this
  analogWrite(TUSS_CLK_PIN, 128); // 50% duty
}

void sendUltrasonicBurst() {
  analogWrite(TUSS_CLK_PIN, 128); // enable waveform
  delayMicroseconds(PING_LENGTH_US);
  analogWrite(TUSS_CLK_PIN, 0);   // stop waveform
}

int getAdaptiveThreshold() {
  int sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(26);
  }
  int noise = sum / 50;
  return noise + 100;   // margin
}

long measureEchoTime() {
  delayMicroseconds(300); // blanking

  unsigned long start = micros();

  while (analogRead(ECHO_ADC_PIN) < THRESHOLD) {
    if (micros() - start > MAX_TIMEOUT) {
      return -1;
    }
  }

  return micros() - start;
}

void loop() {
  sendUltrasonicBurst();
  long dt = measureEchoTime();
  if (dt >= 0) {
    // distance in meters
    float distance = (1480.0f * dt * 1e-6f) / 2.0f;
    Serial.printf("Distance: %.3f m\n", distance);
  }
}
