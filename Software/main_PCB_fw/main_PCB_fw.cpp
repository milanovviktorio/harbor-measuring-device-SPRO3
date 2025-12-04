#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "lib/IMU_lib/ICM42688.hpp"

#include <RF24.h>

// SPI Defines
#define SPI_PORT spi0
#define PIN_MISO 16
//#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 10
#define I2C_SCL 11

// BLDC driver defines
#define CLS 20
#define CHS 21
#define BLS 22
#define BHS 23
#define ALS 24
#define AHS 25

/* DEVELOPMENT CHECKLIST
    This shit needs to:
    * Receive commands from the RC remote
    * Send info back to the remote
    * Control the main motor and the rudder
    * Controlling the main motor specifically entails strobing the phases
    * Take measurements from the gyro
    * Take measurements of the bottom from the sonar
*/

void motor_setup();

void setup_pwm(uint pin, uint freq);

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    //gpio_set_dir(PIN_CS, GPIO_OUT);
    //gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}

void commutate(int step) {
    switch(step) {
        case 0:
            motor_pwm_setup(ALS, AHS, 20000, 0.8);
            motor_pwm_setup(BLS, BHS, 20000, 0.2);
            motor_pwm_setup(CLS, CHS, 20000, 0.0);
            break;
        case 1:
            motor_pwm_setup(ALS, AHS, 20000, 0.8);
            motor_pwm_setup(BLS, BHS, 20000, 0.0);
            motor_pwm_setup(CLS, CHS, 20000, 0.2);
            break;
        case 2:
            motor_pwm_setup(ALS, AHS, 20000, 0.2);
            motor_pwm_setup(BLS, BHS, 20000, 0.0);
            motor_pwm_setup(CLS, CHS, 20000, 0.2);
            break;
        case 3:
            motor_pwm_setup(ALS, AHS, 20000, 0.0);
            motor_pwm_setup(BLS, BHS, 20000, 0.2);
            motor_pwm_setup(CLS, CHS, 20000, 0.8);
            break;
        case 4:
            motor_pwm_setup(ALS, AHS, 20000, 0.0);
            motor_pwm_setup(BLS, BHS, 20000, 0.8);
            motor_pwm_setup(CLS, CHS, 20000, 0.2);
            break;
        case 5:
            motor_pwm_setup(ALS, AHS, 20000, 0.2);
            motor_pwm_setup(BLS, BHS, 20000, 0.8);
            motor_pwm_setup(CLS, CHS, 20000, 0.0);
            break;
    }
}

void motor_pwm_setup(uint pin, uint pin_comp, uint freq, float duty_cycle) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function(pin_comp, GPIO_FUNC_PWM);
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