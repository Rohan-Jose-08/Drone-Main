extern "C"{
#include "bno055.h"
}
#include "hardware/i2c.h"



// I2C configuration
#define I2C_PORT i2c0
#define SDA_PIN 4      // Adjust to your wiring
#define SCL_PIN 5
#define I2C_FREQ 400000

s8 bno055_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 bno055_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

void bno055_delay_msek(u32 msek);