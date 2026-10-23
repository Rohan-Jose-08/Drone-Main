#include <math.h>
#include "pico/stdlib.h"
#include "include/baro.h"

// Registers, from the BMP388/BMP390 datasheets and Bosch's BMP3 driver.
#define REG_CHIP_ID 0x00
#define REG_ERR 0x02
#define REG_STATUS 0x03
#define REG_DATA 0x04              // pressure (3 bytes) then temperature (3 bytes), lowest byte first
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define REG_ODR 0x1D
#define REG_CONFIG 0x1F
#define REG_CALIBRATION 0x31
#define REG_CMD 0x7E

#define CHIP_ID_BMP388 0x50
#define CHIP_ID_BMP390 0x60
#define CMD_SOFT_RESET 0xB6
#define STATUS_PRESSURE_READY 0x20 // cleared when the pressure data is read
#define ERR_CONFIG 0x04            // the settings below don't fit together

// Bosch's settings for drones: pressure oversampled 8 times and temperature once, IIR filter coefficient 3,
// 50 measurements a second. Each measurement takes about 19 ms, inside the 20 ms period.
#define OSR_SETTING 0x03           // pressure x8 (bits 0-2), temperature x1 (bits 3-5)
#define ODR_SETTING 0x02           // 50 Hz
#define CONFIG_SETTING (0x02 << 1) // IIR filter coefficient 3
#define PWR_CTRL_NORMAL 0x33       // pressure and temperature on, measuring continuously

#define MIN_PRESSURE_PA 30000.0    // the sensor's range; anything outside it is a bad reading
#define MAX_PRESSURE_PA 125000.0
#define I2C_TIMEOUT_US 2000        // a stuck sensor can't hold up the control loop for longer than this

static i2c_inst_t *bus;
static uint8_t address;
static BaroCalibration calibration;

static bool read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_write_timeout_us(bus, address, &reg, 1, true, I2C_TIMEOUT_US) == 1 &&
           i2c_read_timeout_us(bus, address, data, length, false, I2C_TIMEOUT_US) == (int)length;
}

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    return i2c_write_timeout_us(bus, address, bytes, 2, false, I2C_TIMEOUT_US) == 2;
}

static uint16_t read_uint16(const uint8_t *bytes)
{
    return (uint16_t)((bytes[1] << 8) | bytes[0]);
}

static uint32_t read_uint24(const uint8_t *bytes)
{
    return bytes[0] | (bytes[1] << 8) | ((uint32_t)bytes[2] << 16);
}

BaroCalibration baro_parse_calibration(const uint8_t bytes[21])
{
    BaroCalibration cal;
    cal.t1 = ldexp(read_uint16(&bytes[0]), 8);
    cal.t2 = ldexp(read_uint16(&bytes[2]), -30);
    cal.t3 = ldexp((int8_t)bytes[4], -48);
    cal.p1 = ldexp((int16_t)read_uint16(&bytes[5]) - 16384, -20);
    cal.p2 = ldexp((int16_t)read_uint16(&bytes[7]) - 16384, -29);
    cal.p3 = ldexp((int8_t)bytes[9], -32);
    cal.p4 = ldexp((int8_t)bytes[10], -37);
    cal.p5 = ldexp(read_uint16(&bytes[11]), 3);
    cal.p6 = ldexp(read_uint16(&bytes[13]), -6);
    cal.p7 = ldexp((int8_t)bytes[15], -8);
    cal.p8 = ldexp((int8_t)bytes[16], -15);
    cal.p9 = ldexp((int16_t)read_uint16(&bytes[17]), -48);
    cal.p10 = ldexp((int8_t)bytes[19], -48);
    cal.p11 = ldexp((int8_t)bytes[20], -65);
    return cal;
}

double baro_compensate(const BaroCalibration &cal, uint32_t rawPressure, uint32_t rawTemperature)
{
    // Temperature first (deg C), since the pressure correction depends on it.
    double fromT1 = (double)rawTemperature - cal.t1;
    double t = fromT1 * cal.t2 + fromT1 * fromT1 * cal.t3;

    double p = rawPressure;
    double offset = cal.p5 + cal.p6 * t + cal.p7 * t * t + cal.p8 * t * t * t;
    double sensitivity = cal.p1 + cal.p2 * t + cal.p3 * t * t + cal.p4 * t * t * t;
    return offset + p * sensitivity + p * p * (cal.p9 + cal.p10 * t) + p * p * p * cal.p11;
}

float pressure_to_altitude(float pressurePa)
{
    return 44330.0f * (1.0f - powf(pressurePa / 101325.0f, 0.190295f));
}

// Resets the sensor, reads its calibration and starts it measuring.
static bool start()
{
    uint8_t bytes[21];
    if (!write_register(REG_CMD, CMD_SOFT_RESET))
    {
        return false;
    }
    sleep_ms(10);
    if (!read_registers(REG_CALIBRATION, bytes, sizeof(bytes)))
    {
        return false;
    }
    calibration = baro_parse_calibration(bytes);

    uint8_t error, mode;
    return write_register(REG_OSR, OSR_SETTING) && write_register(REG_ODR, ODR_SETTING) &&
           write_register(REG_CONFIG, CONFIG_SETTING) && write_register(REG_PWR_CTRL, PWR_CTRL_NORMAL) &&
           read_registers(REG_ERR, &error, 1) && !(error & ERR_CONFIG) &&
           read_registers(REG_PWR_CTRL, &mode, 1) && (mode & PWR_CTRL_NORMAL) == PWR_CTRL_NORMAL;
}

bool baro_init(i2c_inst_t *i2c)
{
    bus = i2c;
    const uint8_t addresses[] = {0x77, 0x76};    // the SDO pin picks one; most breakout boards use 0x77
    for (uint8_t candidate : addresses)
    {
        address = candidate;
        uint8_t id;
        if (read_registers(REG_CHIP_ID, &id, 1) && (id == CHIP_ID_BMP388 || id == CHIP_ID_BMP390))
        {
            return start();
        }
    }
    return false;
}

int baro_read(float &pressurePa)
{
    uint8_t status;
    if (!read_registers(REG_STATUS, &status, 1))
    {
        return -1;
    }
    if (!(status & STATUS_PRESSURE_READY))
    {
        return 0;
    }
    // One burst read, so the pressure and temperature come from the same measurement.
    uint8_t data[6];
    if (!read_registers(REG_DATA, data, sizeof(data)))
    {
        return -1;
    }
    double pressure = baro_compensate(calibration, read_uint24(&data[0]), read_uint24(&data[3]));
    if (pressure < MIN_PRESSURE_PA || pressure > MAX_PRESSURE_PA)
    {
        return -1;
    }
    pressurePa = (float)pressure;
    return 1;
}
