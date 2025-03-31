#include "include/bno.h"





s8 bno055_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    // Create a buffer that contains the register address followed by data bytes.
    uint8_t buffer[cnt + 1];
    buffer[0] = reg_addr;
    for (int i = 0; i < cnt; i++) {
        buffer[i + 1] = reg_data[i];
    }
    int ret = i2c_write_blocking(I2C_PORT, dev_addr, buffer, cnt + 1, false);
    return (ret < 0) ? ret : 0;
}

// Implement the I2C bus read function expected by the Bosch library.
s8 bno055_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    // Write the register address, then read the data.
    int ret = i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true);
    if (ret < 0) return ret;
    ret = i2c_read_blocking(I2C_PORT, dev_addr, reg_data, cnt, false);
    return (ret < 0) ? ret : 0;
}

// Implement the delay function.
void bno055_delay_msek(u32 msek) {
    sleep_ms(msek);
}