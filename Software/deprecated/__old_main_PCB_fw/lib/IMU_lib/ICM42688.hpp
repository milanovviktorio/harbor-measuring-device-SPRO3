#ifndef ICM42688_H
#define ICM42688_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <string>
#include <map>

// Include Pico SDK headers
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

// JSON library
#include "json.hpp"
using json = nlohmann::json;

#define DECIMAL_PLACES 3

// Contains all register address definitions
namespace ICM42688REG
{
    // User Bank 0 registers
    static constexpr uint8_t REG_BANK_SEL = 0x76;
        // 000: Bank 0 (default)
        // 001: Bank 1
        // 010: Bank 2
        // 011: Bank 3
        // 100: Bank 4

    // Configuration registers
    static constexpr uint8_t DEVICE_CONFIG = 0x11;
    static constexpr uint8_t DRIVE_CONFIG = 0x13;
    static constexpr uint8_t INT_CONFIG = 0x14;
    static constexpr uint8_t FIFO_CONFIG = 0x16;
    // Temperature data registers
    static constexpr uint8_t TEMP_DATA1 = 0x1D;
    static constexpr uint8_t TEMP_DATA0 = 0x1E;
    // Accelerometer and Gyroscope data registers
    static constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
    static constexpr uint8_t ACCEL_DATA_X0 = 0x20;
    static constexpr uint8_t ACCEL_DATA_Y1 = 0x21;
    static constexpr uint8_t ACCEL_DATA_Y0 = 0x22;
    static constexpr uint8_t ACCEL_DATA_Z1 = 0x23;
    static constexpr uint8_t ACCEL_DATA_Z0 = 0x24;
    static constexpr uint8_t GYRO_DATA_X1 = 0x25;
    static constexpr uint8_t GYRO_DATA_X0 = 0x26;
    static constexpr uint8_t GYRO_DATA_Y1 = 0x27;
    static constexpr uint8_t GYRO_DATA_Y0 = 0x28;
    static constexpr uint8_t GYRO_DATA_Z1 = 0x29;
    static constexpr uint8_t GYRO_DATA_Z0 = 0x2A;
    // TMST_FSYNC register
    static constexpr uint8_t TMST_FSYNCH = 0x2B;
    static constexpr uint8_t TMST_FSYNCL = 0x2C;
    static constexpr uint8_t INT_STATUS = 0x2D;
    // FIFO registers
    static constexpr uint8_t FIFO_COUNTH = 0x2E;
    static constexpr uint8_t FIFO_COUNTL = 0x2F;
    static constexpr uint8_t FIFO_DATA = 0x30;
    // Apex registers
    static constexpr uint8_t APEX_DATA0 = 0x31;
    static constexpr uint8_t APEX_DATA1 = 0x32;
    static constexpr uint8_t APEX_DATA2 = 0x33;
    static constexpr uint8_t APEX_DATA3 = 0x34;
    static constexpr uint8_t APEX_DATA4 = 0x35;
    static constexpr uint8_t APEX_DATA5 = 0x36;
    // Interrupt status registers
    static constexpr uint8_t INT_STATUS2 = 0x37;
    static constexpr uint8_t INT_STATUS3 = 0x38;
    static constexpr uint8_t SIGNAL_PATH_RESET = 0x4B;
    static constexpr uint8_t INTF_CONFIG0 = 0x4C;
    static constexpr uint8_t INTF_CONFIG1 = 0x4D;
    static constexpr uint8_t PWR_MGMT0 = 0x4E;
    // Configuration registers - Accel and Gyro
    static constexpr uint8_t GYRO_CONFIG0 = 0x4F;
    static constexpr uint8_t ACCEL_CONFIG0 = 0x50;
    static constexpr uint8_t GYRO_CONFIG1 = 0x51;
    static constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr uint8_t ACCEL_CONFIG1 = 0x53;
    // Configuration registers - TMST, Apex, SMD, FIFO, FSYNC
    static constexpr uint8_t TMST_CONFIG = 0x54;
    static constexpr uint8_t APEX_CONFIG0 = 0x56;
    static constexpr uint8_t SMD_CONFIG = 0x57;
    static constexpr uint8_t FIFO_CONFIG1 = 0x5F;
    static constexpr uint8_t FIFO_CONFIG2 = 0x60;
    static constexpr uint8_t FIFO_CONFIG3 = 0x61;
    static constexpr uint8_t FSYNC_CONFIG = 0x62;
    // Interrupt configuration registers
    static constexpr uint8_t INT_CONFIG0 = 0x63;
    static constexpr uint8_t INT_CONFIG1 = 0x64;
    static constexpr uint8_t INT_SOURCE0 = 0x65;
    static constexpr uint8_t INT_SOURCE1 = 0x66;
    static constexpr uint8_t INT_SOURCE3 = 0x68;
    static constexpr uint8_t INT_SOURCE4 = 0x69;

    static constexpr uint8_t FIFO_LOST_PKT0 = 0x6C;
    static constexpr uint8_t FIFO_LOST_PKT1 = 0x6D;
    static constexpr uint8_t SELF_TEST_CONFIG = 0x70;
    static constexpr uint8_t WHO_AM_I = 0x75;

    // User Bank 1 registers
    // Sensor configuration registers
    static constexpr uint8_t SENSOR_CONFIG0 = 0x03;
    // Gyro static configuration registers
    static constexpr uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    static constexpr uint8_t GYRO_CONFIG_STATIC3 = 0x0C;
    static constexpr uint8_t GYRO_CONFIG_STATIC4 = 0x0D;
    static constexpr uint8_t GYRO_CONFIG_STATIC5 = 0x0E;
    static constexpr uint8_t GYRO_CONFIG_STATIC6 = 0x0F;
    static constexpr uint8_t GYRO_CONFIG_STATIC7 = 0x10;
    static constexpr uint8_t GYRO_CONFIG_STATIC8 = 0x11;
    static constexpr uint8_t GYRO_CONFIG_STATIC9 = 0x12;
    static constexpr uint8_t GYRO_CONFIG_STATIC10 = 0x13;
    // X, Y, Z gyro data registers
    static constexpr uint8_t XG_ST_DATA = 0x5F;
    static constexpr uint8_t YG_ST_DATA = 0x60;
    static constexpr uint8_t ZG_ST_DATA = 0x61;
    // Time stamp registers
    static constexpr uint8_t TMSTVAL0 = 0x62;
    static constexpr uint8_t TMSTVAL1 = 0x63;
    static constexpr uint8_t TMSTVAL2 = 0x64;
    // Interrupt configuration registers
    static constexpr uint8_t INTF_CONFIG4 = 0x7A;
    static constexpr uint8_t INTF_CONFIG5 = 0x7B;
    static constexpr uint8_t INTF_CONFIG6 = 0x7C;

    // User Bank 2 registers
    static constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    static constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
    static constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
    // X, Y, Z accel data registers
    static constexpr uint8_t XA_ST_DATA = 0x3B;
    static constexpr uint8_t YA_ST_DATA = 0x3C;
    static constexpr uint8_t ZA_ST_DATA = 0x3D;

    // User Bank 3 registers
    // Clock divider configuration registers
    static constexpr uint8_t CLKDIV = 0x2A;

    // User Bank 4 registers
    // Apex configuration registers
    static constexpr uint8_t APEX_CONFIG1 = 0x40;
    static constexpr uint8_t APEX_CONFIG2 = 0x41;
    static constexpr uint8_t APEX_CONFIG3 = 0x42;
    static constexpr uint8_t APEX_CONFIG4 = 0x43;
    static constexpr uint8_t APEX_CONFIG5 = 0x44;
    static constexpr uint8_t APEX_CONFIG6 = 0x45;
    static constexpr uint8_t APEX_CONFIG7 = 0x46;
    static constexpr uint8_t APEX_CONFIG8 = 0x47;
    static constexpr uint8_t APEX_CONFIG9 = 0x48;
    // Accel WOM registers
    static constexpr uint8_t ACCEL_WOM_X_THR = 0x4A;
    static constexpr uint8_t ACCEL_WOM_Y_THR = 0x4B;
    static constexpr uint8_t ACCEL_WOM_Z_THR = 0x4C;
    // Interrupt source registers
    static constexpr uint8_t INT_SOURCE6 = 0x4D;
    static constexpr uint8_t INT_SOURCE7 = 0x4E;
    static constexpr uint8_t INT_SOURCE8 = 0x4F;
    static constexpr uint8_t INT_SOURCE9 = 0x50;
    static constexpr uint8_t INT_SOURCE10 = 0x51;
    // Offset registers
    static constexpr uint8_t OFFSET_USER0 = 0x77;
    static constexpr uint8_t OFFSET_USER1 = 0x78;
    static constexpr uint8_t OFFSET_USER2 = 0x79;
    static constexpr uint8_t OFFSET_USER3 = 0x7A;
    static constexpr uint8_t OFFSET_USER4 = 0x7B;
    static constexpr uint8_t OFFSET_USER5 = 0x7C;
    static constexpr uint8_t OFFSET_USER6 = 0x7D;
    static constexpr uint8_t OFFSET_USER7 = 0x7E;
    static constexpr uint8_t OFFSET_USER8 = 0x7F;
}

// Contains all setting value definitions
namespace ICM42688SET
{
    // GYRO_FS_SEL DEG/SEC
    static constexpr uint8_t GYRO_FS_SEL_2000   = 0b000;
    static constexpr uint8_t GYRO_FS_SEL_1000   = 0b001;
    static constexpr uint8_t GYRO_FS_SEL_500    = 0b010;
    static constexpr uint8_t GYRO_FS_SEL_250    = 0b011;
    static constexpr uint8_t GYRO_FS_SEL_125    = 0b100;
    static constexpr uint8_t GYRO_FS_SEL_62     = 0b101;
    static constexpr uint8_t GYRO_FS_SEL_31     = 0b110;
    static constexpr uint8_t GYRO_FS_SEL_15     = 0b111;

    // GYRO_ODR Hz
    static constexpr uint8_t GYRO_ODR_32K       = 0b0001;
    static constexpr uint8_t GYRO_ODR_16K       = 0b0010;
    static constexpr uint8_t GYRO_ODR_8K        = 0b0011;
    static constexpr uint8_t GYRO_ODR_4K        = 0b0100;
    static constexpr uint8_t GYRO_ODR_2K        = 0b0101;
    static constexpr uint8_t GYRO_ODR_1K        = 0b0110;
    static constexpr uint8_t GYRO_ODR_200       = 0b0111;
    static constexpr uint8_t GYRO_ODR_100       = 0b1000;
    static constexpr uint8_t GYRO_ODR_50        = 0b1001;
    static constexpr uint8_t GYRO_ODR_25        = 0b1010;
    static constexpr uint8_t GYRO_ODR_12        = 0b1011;
    static constexpr uint8_t GYRO_ODR_500       = 0b1111;

    // ACCEL_FS_SEL g
    static constexpr uint8_t ACCEL_FS_SEL_16    = 0b000;
    static constexpr uint8_t ACCEL_FS_SEL_8     = 0b001;
    static constexpr uint8_t ACCEL_FS_SEL_4     = 0b010;
    static constexpr uint8_t ACCEL_FS_SEL_2     = 0b011;

    // ACCEL_ODR Hz
    static constexpr uint8_t ACCEL_ODR_32K      = 0b0001;
    static constexpr uint8_t ACCEL_ODR_16K      = 0b0010;
    static constexpr uint8_t ACCEL_ODR_8K       = 0b0011;
    static constexpr uint8_t ACCEL_ODR_4K       = 0b0100;
    static constexpr uint8_t ACCEL_ODR_2K       = 0b0101;
    static constexpr uint8_t ACCEL_ODR_1K       = 0b0110;
    static constexpr uint8_t ACCEL_ODR_200      = 0b0111;
    static constexpr uint8_t ACCEL_ODR_100      = 0b1000;
    static constexpr uint8_t ACCEL_ODR_50       = 0b1001;
    static constexpr uint8_t ACCEL_ODR_25       = 0b1010;
    static constexpr uint8_t ACCEL_ODR_12       = 0b1011;
    static constexpr uint8_t ACCEL_ODR_6        = 0b1100;
    static constexpr uint8_t ACCEL_ODR_3        = 0b1101;
    static constexpr uint8_t ACCEL_ODR_1        = 0b1110;
    static constexpr uint8_t ACCEL_ODR_500      = 0b1111;
}

/* ----- Sensor Settings Structures ----- */

// Universal struct for settings
struct SensorSetting {
    uint8_t value;
    std::string name;
};

// Boolean setting struct
struct BoolSetting {
    bool enabled;
    std::string name;
};

// Anti-Aliasing Filter Setting struct
struct AAFSetting {
    BoolSetting enabled;  // AAF Enable/Disable
    uint8_t delt;
    uint16_t deltsqr;
    uint8_t bitshift;
};

// Power Modes for Accelerometer
static const SensorSetting AccelPowerModes[] = {
    {0x00, "Accelerometer Off (default)"},  // 00: Turns accelerometer off (default)
    {0x02, "Low Power (LP) Mode"},          // 10: Places accelerometer in Low Power (LP) Mode
    {0x03, "Low Noise (LN) Mode"}           // 11: Places accelerometer in Low Noise (LN) Mode
};

// Power Modes for Gyroscope
static const SensorSetting GyroPowerModes[] = {
    {0x00, "Gyroscope Off (default)"},      // 00: Turns gyroscope off (default)
    {0x01, "Standby Mode"},                 // 01: Places gyroscope in Standby Mode
    {0x03, "Low Noise (LN) Mode"}           // 11: Places gyroscope in Low Noise (LN) Mode
};

// Clock Sources
static const SensorSetting ClockSources[] = {
    {0x00, "Internal"},
    {0x01, "Auto"},
    {0x02, "Reserved"},
    {0x03, "External"}
};

// Accelerometer ODR settings
static const SensorSetting AccelODRSettings[] = {
    {ICM42688SET::ACCEL_ODR_32K, "32kHz"}, //0 LN
    {ICM42688SET::ACCEL_ODR_16K, "16kHz"}, //1 LN
    {ICM42688SET::ACCEL_ODR_8K, "8kHz"}, //2 LN
    {ICM42688SET::ACCEL_ODR_4K, "4kHz"}, //3 LN
    {ICM42688SET::ACCEL_ODR_2K, "2kHz"}, //4 LN
    {ICM42688SET::ACCEL_ODR_1K, "1kHz"}, //5  LN
    {ICM42688SET::ACCEL_ODR_200, "200Hz"}, //6 LN or LP
    {ICM42688SET::ACCEL_ODR_100, "100Hz"}, //7 LN or LP
    {ICM42688SET::ACCEL_ODR_50, "50Hz"}, //8 LN or LP
    {ICM42688SET::ACCEL_ODR_25, "25Hz"}, //9 LN or LP
    {ICM42688SET::ACCEL_ODR_12, "12.5Hz"}, //10 LN or LP
    {ICM42688SET::ACCEL_ODR_6, "6.25Hz"}, //11 LP
    {ICM42688SET::ACCEL_ODR_3, "3.125Hz"}, //12 LP
    {ICM42688SET::ACCEL_ODR_1, "1.5625Hz"}, //13 LP
    {ICM42688SET::ACCEL_ODR_500, "500Hz"} //14 LN or LP
};

// Accelerometer FSR settings
static const SensorSetting AccelFSRSettings[] = {
    {ICM42688SET::ACCEL_FS_SEL_16, "±16g"},
    {ICM42688SET::ACCEL_FS_SEL_8, "±8g"},
    {ICM42688SET::ACCEL_FS_SEL_4, "±4g"},
    {ICM42688SET::ACCEL_FS_SEL_2, "±2g"}
};

// Gyroscope ODR settings
static const SensorSetting GyroODRSettings[] = {
    {ICM42688SET::GYRO_ODR_32K, "32kHz"}, //0
    {ICM42688SET::GYRO_ODR_16K, "16kHz"}, //1
    {ICM42688SET::GYRO_ODR_8K, "8kHz"},  //2
    {ICM42688SET::GYRO_ODR_4K, "4kHz"}, //3
    {ICM42688SET::GYRO_ODR_2K, "2kHz"}, //4
    {ICM42688SET::GYRO_ODR_1K, "1kHz"}, //5
    {ICM42688SET::GYRO_ODR_200, "200Hz"}, //6
    {ICM42688SET::GYRO_ODR_100, "100Hz"}, //7
    {ICM42688SET::GYRO_ODR_50, "50Hz"}, //8
    {ICM42688SET::GYRO_ODR_25, "25Hz"}, //9
    {ICM42688SET::GYRO_ODR_12, "12.5Hz"}, //10
    {ICM42688SET::GYRO_ODR_500, "500Hz"} //11
};

// Gyroscope FSR settings
static const SensorSetting GyroFSRSettings[] = {
    {ICM42688SET::GYRO_FS_SEL_2000, "±2000dps"},
    {ICM42688SET::GYRO_FS_SEL_1000, "±1000dps"},
    {ICM42688SET::GYRO_FS_SEL_500, "±500dps"},
    {ICM42688SET::GYRO_FS_SEL_250, "±250dps"},
    {ICM42688SET::GYRO_FS_SEL_125, "±125dps"},
    {ICM42688SET::GYRO_FS_SEL_62, "±62.5dps"},
    {ICM42688SET::GYRO_FS_SEL_31, "±31.25dps"},
    {ICM42688SET::GYRO_FS_SEL_15, "±15.625dps"}
};

// Mapping of FSR to sensitivity for accelerometer and gyroscope
static std::map<uint8_t, float> accel_sensitivity_map = {
    {ICM42688SET::ACCEL_FS_SEL_2, 16384.0f},
    {ICM42688SET::ACCEL_FS_SEL_4, 8192.0f},
    {ICM42688SET::ACCEL_FS_SEL_8, 4096.0f},
    {ICM42688SET::ACCEL_FS_SEL_16, 2048.0f}
};

static std::map<uint8_t, float> gyro_sensitivity_map = {
    {ICM42688SET::GYRO_FS_SEL_2000, 16.4f},
    {ICM42688SET::GYRO_FS_SEL_1000, 32.8f},
    {ICM42688SET::GYRO_FS_SEL_500, 65.5f},
    {ICM42688SET::GYRO_FS_SEL_250, 131.0f},
    {ICM42688SET::GYRO_FS_SEL_125, 262.0f},
    {ICM42688SET::GYRO_FS_SEL_62, 524.3f},
    {ICM42688SET::GYRO_FS_SEL_31, 1048.6f},
    {ICM42688SET::GYRO_FS_SEL_15, 2097.2f}
};

/* ----- Output Data Structures ----- */

// Data structures for sensor data
struct AccelerometerData {
    float x;
    float y;
    float z;
};

struct GyroscopeData {
    float x;
    float y;
    float z;
};

struct TemperatureData {
    float celcius;
};

struct SensorData {
    AccelerometerData accel;
    GyroscopeData gyro;
    TemperatureData temp;
    uint32_t imu_timestamp; // IMU timestamp (24-bit)
};

class IMU {
public:
    // Constructor
    IMU(spi_inst_t* spi_port, uint cs_pin, uint8_t subdevice_id);

    // Initialize the IMU
    void initialize();

    // Read sensor data
    SensorData read_sensor_data();

    // Generate JSON data
    // json jsonify_data(const SensorData& data_in);
    std::string jsonify_data(const SensorData& data_in);

    // Generate JSON settings
    json jsonify_settings();

    // Read IMU timestamp
    uint32_t read_imu_timestamp();

    void set_accel_odr(const uint odr);
    void set_accel_fsr(const uint fsr);
    void set_gyro_odr(const uint odr);
    void set_gyro_fsr(const uint fsr);

private:
    // SPI variables
    spi_inst_t* spi_port_;
    uint cs_pin_;
    uint8_t subdevice_id_;

    // Settings
    SensorSetting accel_odr_setting_;
    SensorSetting accel_fsr_setting_;
    SensorSetting gyro_odr_setting_;
    SensorSetting gyro_fsr_setting_;
    uint defaultFSR_;
    uint defaultODR_;

    // Sensitivity values
    float accel_sensitivity_;
    float gyro_sensitivity_;

    // Power modes
    SensorSetting accel_power_mode_;
    SensorSetting gyro_power_mode_;
    bool temp_disabled_;

    // Clock settings
    SensorSetting clock_source_setting_;
    bool rtc_mode_enabled_;

    // Timestamp enabled
    bool timestamp_enabled_;

    // Filter settings
    AAFSetting accel_aaf_setting_;
    AAFSetting gyro_aaf_setting_;

    // Private methods
    void cs_select();
    void cs_deselect();
    void write_register(uint8_t reg_addr, uint8_t data);
    uint8_t read_register(uint8_t reg_addr);
    void read_registers(uint8_t reg_addr, uint8_t* data, size_t length);
    void select_register_bank(uint8_t bank);
    void print_register(uint8_t reg_addr);
    void setup_spi();
    void reset_device_configuration();
    void configure_sensor();
    AccelerometerData read_accel();
    GyroscopeData read_gyro();
    TemperatureData read_temp();
    void calculate_sensitivity();
    void set_power_modes(const SensorSetting& accel_mode, const SensorSetting& gyro_mode, bool temp_disabled);
    void set_clock();
};


#endif // ICM42688_HPP
