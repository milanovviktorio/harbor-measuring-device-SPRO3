unsigned int motor_step;

void motor_commutate(int step);

void motor_pwm_setup();

void motor_pwm_startup();

void motor_pwm_state(uint pin, uint pin_comp, uint freq, float duty_cycle);

void motor_commutate(int step) {
    AS1_val=1;
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
  delay(100);
  motor_pwm_state(ALS, AHS, 20000, 0.8);
  motor_pwm_state(BLS, BHS, 20000, 0.0);
  motor_pwm_state(CLS, CHS, 20000, 0.2);
  delay(100);
  motor_pwm_state(ALS, AHS, 20000, 0.2);
  motor_pwm_state(BLS, BHS, 20000, 0.0);
  motor_pwm_state(CLS, CHS, 20000, 0.2);
  delay(100);
  motor_pwm_state(ALS, AHS, 20000, 0.0);
  motor_pwm_state(BLS, BHS, 20000, 0.2);
  motor_pwm_state(CLS, CHS, 20000, 0.8);
  delay(100);
  motor_pwm_state(ALS, AHS, 20000, 0.0);
  motor_pwm_state(BLS, BHS, 20000, 0.8);
  motor_pwm_state(CLS, CHS, 20000, 0.2);
  delay(100);
  motor_pwm_state(ALS, AHS, 20000, 0.2);
  motor_pwm_state(BLS, BHS, 20000, 0.8);
  motor_pwm_state(CLS, CHS, 20000, 0.0);
  motor_step=5;
  delay(100);
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