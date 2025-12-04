#include "ICM42688.hpp"

#include <math.h>    // For pow and roundf
#include <stdio.h>   // For snprintf
// Constructor
IMU::IMU(spi_inst_t* spi_port, uint cs_pin, uint8_t subdevice_id)
    : spi_port_(spi_port), cs_pin_(cs_pin), subdevice_id_(subdevice_id) {
    defaultFSR_ = 2; // ±4g accel, ±500 dps gyro
    defaultODR_ = 6; // 12.5 Hz
    initialize();
}

// Initialize the IMU
void IMU::initialize() {
    setup_spi();
    configure_sensor();
    uint8_t who_am_i = read_register(ICM42688REG::WHO_AM_I);
    if (who_am_i != 0x47) { // Assuming 0x47 is the expected WHO_AM_I value
        printf("IMU %d not detected! WHO_AM_I: 0x%02X\n", subdevice_id_, who_am_i);
    } else {
        printf("IMU %d detected successfully. WHO_AM_I: 0x%02X\n", subdevice_id_, who_am_i);
    }
}

// Read sensor data
SensorData IMU::read_sensor_data() {
    // printf("Reading sensor data from IMU %d\n", subdevice_id_);
    SensorData data;
    data.accel = read_accel();
    data.gyro = read_gyro();
    data.temp = read_temp();
    data.imu_timestamp = read_imu_timestamp();
    return data;
}


// Generate JSON data
std::string IMU::jsonify_data(const SensorData& data_in) {
    char buffer[512];

    // Construct JSON string manually with limited decimal places
    int written = snprintf(buffer, sizeof(buffer),
             "{\"subdevice\":\"%d\","
             "\"timestamp\":\"%08x\","
             "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"temperature\":%.3f}",
             subdevice_id_,
             data_in.imu_timestamp,
             data_in.accel.x, data_in.accel.y, data_in.accel.z,
             data_in.gyro.x, data_in.gyro.y, data_in.gyro.z,
             data_in.temp.celcius);

    return std::string(buffer);
}


// Generate JSON settings
json IMU::jsonify_settings() {
    json settings_data_json;
    settings_data_json["settings"]["subdevice"] = subdevice_id_;

    settings_data_json["settings"]["accel_odr"] = accel_odr_setting_.name;
    settings_data_json["settings"]["accel_fsr"] = accel_fsr_setting_.name;
    settings_data_json["settings"]["accel_sensitivity"] = accel_sensitivity_;

    settings_data_json["settings"]["gyro_odr"] = gyro_odr_setting_.name;
    settings_data_json["settings"]["gyro_fsr"] = gyro_fsr_setting_.name;
    settings_data_json["settings"]["gyro_sensitivity"] = gyro_sensitivity_;

    settings_data_json["settings"]["power"]["temp_disabled"] = temp_disabled_;
    settings_data_json["settings"]["power"]["accel_mode"] = accel_power_mode_.name;
    settings_data_json["settings"]["power"]["gyro_mode"] = gyro_power_mode_.name;

    settings_data_json["settings"]["clock"]["source"] = clock_source_setting_.name;
    settings_data_json["settings"]["clock"]["rtc_mode_enabled"] = rtc_mode_enabled_;

    settings_data_json["settings"]["timestamp_enabled"] = timestamp_enabled_;

    return settings_data_json;
}

// Read IMU timestamp
uint32_t IMU::read_imu_timestamp() {
    // Strobe the signal path reset register
    uint8_t signal_path_reset_val = 0b01100100;
    write_register(ICM42688REG::SIGNAL_PATH_RESET, signal_path_reset_val);

    // Switch to User Bank 1
    select_register_bank(1);
    uint8_t tmstval0 = read_register(ICM42688REG::TMSTVAL0);
    uint8_t tmstval1 = read_register(ICM42688REG::TMSTVAL1);
    uint8_t tmstval2 = read_register(ICM42688REG::TMSTVAL2);
    // Return to User Bank 0
    select_register_bank(0);

    uint32_t imu_timestamp = ((uint32_t)tmstval2 << 16) | ((uint32_t)tmstval1 << 8) | (uint32_t)tmstval0;
    return imu_timestamp; // 24-bit timestamp
}

/* ----- Private Methods Implementation ----- */

// SPI Communication Methods
void IMU::cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin_, 0); // Active Low
    asm volatile("nop \n nop \n nop");
}

void IMU::cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin_, 1); // Active Low
    asm volatile("nop \n nop \n nop");
}

void IMU::write_register(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buf[2];
    tx_buf[0] = reg_addr & 0x7F; // Clear MSB for write operation
    tx_buf[1] = data;

    cs_select();
    spi_write_blocking(spi_port_, tx_buf, 2);
    cs_deselect();
    // sleep_ms(1);
}

uint8_t IMU::read_register(uint8_t reg_addr) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = reg_addr | 0x80; // Set MSB for read operation
    tx_buf[1] = 0x00;            // Dummy byte

    cs_select();
    spi_write_read_blocking(spi_port_, tx_buf, rx_buf, 2);
    cs_deselect();

    return rx_buf[1]; // The second byte contains the data
}

void IMU::read_registers(uint8_t reg_addr, uint8_t* data, size_t length) {
    uint8_t tx_buf[length + 1];
    uint8_t rx_buf[length + 1];

    tx_buf[0] = reg_addr | 0x80; // Set MSB for read operation
    memset(&tx_buf[1], 0x00, length); // Dummy bytes

    cs_select();
    spi_write_read_blocking(spi_port_, tx_buf, rx_buf, length + 1);
    cs_deselect();

    memcpy(data, &rx_buf[1], length);
}

void IMU::select_register_bank(uint8_t bank) {
    write_register(ICM42688REG::REG_BANK_SEL, bank & 0x07); // Ensure bank is within 0-7
    sleep_us(200);
}

void IMU::print_register(uint8_t reg_addr) {
    uint8_t reg_value = read_register(reg_addr);
    printf("\nRegister 0x%02X: 0x%02X\n", reg_addr, reg_value); // Print in hex
    printf("Register 0x%02X: 0b", reg_addr); // Print in binary
    // for(int i = 7; i >=0; --i){
    //     printf("%u", (reg_value >> i) & 1);
    // }
    // printf("\n");
}

/* ----- SPI Setup ----- */

void IMU::setup_spi() {
    // CS pin setup
    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    gpio_put(cs_pin_, 1); // Deselect the device
}

/* ----- Device Setup ----- */

void IMU::reset_device_configuration(){
    write_register(ICM42688REG::DEVICE_CONFIG, 0x01);
    sleep_ms(10);
}

// Sensor configuration
void IMU::configure_sensor() {
    reset_device_configuration();
    sleep_us(200);
    
    set_clock(); // Uncomment if clock settings are needed
    set_power_modes(GyroPowerModes[2], AccelPowerModes[2], false); // Accel & Gyro Low Noise, Temp Enabled

    // Set custom accelerometer settings
    set_accel_odr(7); // 100Hz
    set_accel_fsr(2); // ±4g

    // Set custom gyroscope settings
    set_gyro_odr(7); // 100Hz
    set_gyro_fsr(2); // ±500dps

    // Calculate sensitivity
    calculate_sensitivity();
}

/* ----- Data Read Methods ----- */

// Read accelerometer data
AccelerometerData IMU::read_accel() {
    uint8_t accel_raw_data[6];
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    float accel_x_g, accel_y_g, accel_z_g;

    read_registers(ICM42688REG::ACCEL_DATA_X1, accel_raw_data, 6);

    accel_x_raw = (int16_t)((accel_raw_data[0] << 8) | accel_raw_data[1]);
    accel_y_raw = (int16_t)((accel_raw_data[2] << 8) | accel_raw_data[3]);
    accel_z_raw = (int16_t)((accel_raw_data[4] << 8) | accel_raw_data[5]);

    accel_x_g = (float)accel_x_raw / accel_sensitivity_;
    accel_y_g = (float)accel_y_raw / accel_sensitivity_;
    accel_z_g = (float)accel_z_raw / accel_sensitivity_;

    return {accel_x_g, accel_y_g, accel_z_g};
}

// Read gyroscope data
GyroscopeData IMU::read_gyro() {
    uint8_t gyro_raw_data[6];
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    read_registers(ICM42688REG::GYRO_DATA_X1, gyro_raw_data, 6);

    gyro_x_raw = (int16_t)((gyro_raw_data[0] << 8) | gyro_raw_data[1]);
    gyro_y_raw = (int16_t)((gyro_raw_data[2] << 8) | gyro_raw_data[3]);
    gyro_z_raw = (int16_t)((gyro_raw_data[4] << 8) | gyro_raw_data[5]);

    gyro_x_dps = (float)gyro_x_raw / gyro_sensitivity_;
    gyro_y_dps = (float)gyro_y_raw / gyro_sensitivity_;
    gyro_z_dps = (float)gyro_z_raw / gyro_sensitivity_;

    return {gyro_x_dps, gyro_y_dps, gyro_z_dps};
}

// Read temperature data
TemperatureData IMU::read_temp() {
    uint8_t temp_raw_data[2];
    int16_t temp_data_raw;
    float temperature_c;

    read_registers(ICM42688REG::TEMP_DATA1, temp_raw_data, 2);

    temp_data_raw = (int16_t)((temp_raw_data[0] << 8) | temp_raw_data[1]);
    temperature_c = ((float)temp_data_raw / 132.48f) + 25.0f;

    return {temperature_c};
}

/* ----- Configuration Methods ----- */

// Set accelerometer ODR
void IMU::set_accel_odr(const uint odr) {
    // Check if the ODR setting is valid
    size_t odr_count = sizeof(AccelODRSettings) / sizeof(AccelODRSettings[0]);
    if (odr >= odr_count) {
        accel_odr_setting_ = AccelODRSettings[defaultODR_];
        printf("Invalid ODR setting. Defaulting to %s\n", accel_odr_setting_.name.c_str());
    } else {
        accel_odr_setting_ = AccelODRSettings[odr];
    }

    // Write the new ODR setting to the register
    uint8_t reg_value = ((uint8_t)(accel_fsr_setting_.value) << 5) | ((uint8_t)(accel_odr_setting_.value) & 0x0F);
    write_register(ICM42688REG::ACCEL_CONFIG0, reg_value);
    calculate_sensitivity();
}

// Set accelerometer FSR
void IMU::set_accel_fsr(const uint fsr) {
    // Check if the FSR setting is valid
    size_t fsr_count = sizeof(AccelFSRSettings) / sizeof(AccelFSRSettings[0]);
    if (fsr >= fsr_count) {
        accel_fsr_setting_ = AccelFSRSettings[defaultFSR_];
        printf("Invalid FSR setting. Defaulting to %s\n", accel_fsr_setting_.name.c_str());
    } else {
        accel_fsr_setting_ = AccelFSRSettings[fsr];
    }

    // Write the new FSR setting to the register
    uint8_t reg_value = ((uint8_t)(accel_fsr_setting_.value) << 5) | ((uint8_t)(accel_odr_setting_.value) & 0x0F);
    write_register(ICM42688REG::ACCEL_CONFIG0, reg_value);
    calculate_sensitivity();
}

// Set gyroscope ODR
void IMU::set_gyro_odr(const uint odr) {
    // Check if the ODR setting is valid
    size_t odr_count = sizeof(GyroODRSettings) / sizeof(GyroODRSettings[0]);
    if (odr >= odr_count) {
        gyro_odr_setting_ = GyroODRSettings[defaultODR_];
        printf("Invalid ODR setting. Defaulting to %s\n", gyro_odr_setting_.name.c_str());
    } else {
        gyro_odr_setting_ = GyroODRSettings[odr];
    }

    // Write the new ODR setting to the register
    uint8_t reg_value = ((uint8_t)(gyro_fsr_setting_.value) << 5) | ((uint8_t)(gyro_odr_setting_.value) & 0x0F);
    write_register(ICM42688REG::GYRO_CONFIG0, reg_value);
    calculate_sensitivity();
}

// Set gyroscope FSR
void IMU::set_gyro_fsr(const uint fsr) {
    // Check if the FSR setting is valid
    size_t fsr_count = sizeof(GyroFSRSettings) / sizeof(GyroFSRSettings[0]);
    if (fsr >= fsr_count) {
        gyro_fsr_setting_ = GyroFSRSettings[defaultFSR_];
        printf("Invalid FSR setting. Defaulting to %s\n", gyro_fsr_setting_.name.c_str());
    } else {
        gyro_fsr_setting_ = GyroFSRSettings[fsr];
    }

    // Write the new FSR setting to the register
    uint8_t reg_value = ((uint8_t)(gyro_fsr_setting_.value) << 5) | ((uint8_t)(gyro_odr_setting_.value) & 0x0F);
    write_register(ICM42688REG::GYRO_CONFIG0, reg_value);
    calculate_sensitivity();
}

// Calculate sensitivity based on FSR
void IMU::calculate_sensitivity() {
    // Accelerometer sensitivity (LSB/g)
    auto accel_it = accel_sensitivity_map.find(accel_fsr_setting_.value);
    if (accel_it != accel_sensitivity_map.end()) {
        accel_sensitivity_ = accel_it->second;
    } else {
        accel_sensitivity_ = accel_sensitivity_map[ICM42688SET::ACCEL_FS_SEL_4]; // Default to ±4g
    }

    // Gyroscope sensitivity (LSB/dps)
    auto gyro_it = gyro_sensitivity_map.find(gyro_fsr_setting_.value);
    if (gyro_it != gyro_sensitivity_map.end()) {
        gyro_sensitivity_ = gyro_it->second;
    } else {
        gyro_sensitivity_ = gyro_sensitivity_map[ICM42688SET::GYRO_FS_SEL_500]; // Default to ±500dps
    }
}

/* ----- Power Management ----- */

// Set power modes
void IMU::set_power_modes(const SensorSetting& accel_mode, const SensorSetting& gyro_mode, bool temp_disabled) {
    temp_disabled_ = temp_disabled;

    uint8_t pwr_mgmt0 = 0;
    pwr_mgmt0 |= (temp_disabled_ ? 0x20 : 0x00); // TEMP_DIS bit (bit 5)
    pwr_mgmt0 |= ((gyro_mode.value & 0x03) << 2); // GYRO_MODE bits (bits 3:2)
    pwr_mgmt0 |= (accel_mode.value & 0x03);       // ACCEL_MODE bits (bits 1:0)
    write_register(ICM42688REG::PWR_MGMT0, pwr_mgmt0);

    // Store settings
    accel_power_mode_ = accel_mode;
    gyro_power_mode_ = gyro_mode;
}

/* ----- Clock Configuration ----- */

void IMU::set_clock() {

    // Configure Pin 9 as CLKIN for external clock
    select_register_bank(1);
    uint8_t intf_config5_val = 0b00000100;  // PIN9_FUNCTION = CLKIN
    write_register(ICM42688REG::INTF_CONFIG5, intf_config5_val);
    select_register_bank(0);


    // Configure TMST_CONFIG to enable timestamps, FSYNC, and delta timestamps
    uint8_t tmst_config_val = 0b00011011;  // Enable TMST_EN | TMST_FSYNC_EN | TMST_RES, without TMST_DELTA_EN
    write_register(ICM42688REG::TMST_CONFIG, tmst_config_val);

    // Latch the timestamp once at the beginning, trigger TMST_STROBE
    uint8_t signal_path_reset_val = 0b01100100;  // TMST_STROBE = 1 | DMP_MEM_RESET_EN | DMP_INIT_EN
    write_register(ICM42688REG::SIGNAL_PATH_RESET, signal_path_reset_val);



}
