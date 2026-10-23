#ifndef BARO_H
#define BARO_H

#include <stdint.h>
#include "hardware/i2c.h"

// BMP388 or BMP390 barometer (they share a register layout), on the same I2C bus as the BNO055.

// Compensation constants from the sensor's calibration memory, scaled as in Bosch's BMP3 driver.
struct BaroCalibration
{
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

// From the 21 calibration bytes starting at register 0x31.
BaroCalibration baro_parse_calibration(const uint8_t bytes[21]);

// Pressure in Pa from the raw 24-bit pressure and temperature readings.
double baro_compensate(const BaroCalibration &cal, uint32_t rawPressure, uint32_t rawTemperature);

// Height above sea level in metres for a pressure, in the standard atmosphere. Weather moves it by tens
// of metres over a day, so only changes over a flight mean anything.
float pressure_to_altitude(float pressurePa);

// Looks for the sensor at both of its addresses and starts it measuring 50 times a second.
// Returns false if there's no sensor or it won't start.
bool baro_init(i2c_inst_t *i2c);

// Returns 1 with pressurePa set when a new measurement is ready, 0 if there isn't one yet,
// and -1 if the sensor didn't answer or gave a pressure outside its range.
int baro_read(float &pressurePa);

#endif
