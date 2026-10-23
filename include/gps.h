#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include "hardware/uart.h"

// u-blox GPS receiver (M8, M9 or M10) on a UART, sending UBX-NAV-PVT messages.

struct GpsFix
{
    uint8_t fixType;            // 0 none, 2 2D, 3 3D (as in UBX-NAV-PVT)
    bool fixOk;                 // the receiver considers it within its accuracy limits
    uint8_t satellites;
    int32_t latitude, longitude;    // degrees x 1e7
    float altitude;             // m above mean sea level
    float velocity[3];          // m/s north, east, down
    float horizontalAccuracy;   // m, the receiver's own estimate
    float speedAccuracy;        // m/s, likewise
};

// Sets the receiver, whatever baud rate it starts at, to 115200 baud, UBX only, a UBX-NAV-PVT message
// every rateMs and the airborne dynamic model, then starts receiving. Takes about half a second. The
// settings aren't saved in the receiver, so this runs at every power-up.
void gps_init(uart_inst_t *uart, uint txPin, uint rxPin, uint rateMs);

// Decodes whatever has arrived. Returns true with fix filled in when a new NAV-PVT message has.
bool gps_read(GpsFix &fix);

// North and east offset in metres of a position from an origin (both in degrees x 1e7).
void gps_offset(int32_t latitude, int32_t longitude, int32_t originLatitude, int32_t originLongitude,
                float &north, float &east);

#endif
