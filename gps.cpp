#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "include/gps.h"

// UBX messages, from the u-blox interface descriptions. M8 receivers only understand the legacy
// configuration messages (CFG-PRT, CFG-MSG, CFG-RATE, CFG-NAV5) and M10 ones only UBX-CFG-VALSET, so both
// are sent; each receiver rejects the kind it doesn't know.
#define CLASS_NAV 0x01
#define ID_NAV_PVT 0x07
#define NAV_PVT_LENGTH 92
#define CLASS_CFG 0x06
#define ID_CFG_PRT 0x00
#define ID_CFG_MSG 0x01
#define ID_CFG_RATE 0x08
#define ID_CFG_NAV5 0x24
#define ID_CFG_VALSET 0x8A

// UBX-CFG-VALSET keys
#define KEY_UART1_BAUDRATE 0x40520001      // U4
#define KEY_UART1OUTPROT_UBX 0x10740001    // L
#define KEY_UART1OUTPROT_NMEA 0x10740002   // L
#define KEY_MSGOUT_NAV_PVT_UART1 0x20910007 // U1
#define KEY_RATE_MEAS 0x30210001           // U2, ms
#define KEY_NAVSPG_DYNMODEL 0x20110021     // E1

#define DYNMODEL_AIRBORNE_1G 6
#define GPS_BAUD 115200
#define RX_BUFFER_SIZE 512                 // a NAV-PVT message is 100 bytes

static uart_inst_t *port;
static volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
static volatile uint32_t rxHead = 0;       // written by the interrupt
static volatile uint32_t rxTail = 0;       // written by gps_read

// Moves received bytes into the ring buffer, so none are lost while the control loop is busy.
static void on_uart_rx()
{
    while (uart_is_readable(port))
    {
        uint8_t byte = (uint8_t)uart_getc(port);
        if (rxHead - rxTail < RX_BUFFER_SIZE)
        {
            rxBuffer[rxHead % RX_BUFFER_SIZE] = byte;
            rxHead++;
        }
    }
}

static void put_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = value & 0xFF;
    bytes[1] = value >> 8;
}

static void put_u32(uint8_t *bytes, uint32_t value)
{
    for (int i = 0; i < 4; i++)
    {
        bytes[i] = (value >> (8 * i)) & 0xFF;
    }
}

static uint32_t get_u32(const uint8_t *bytes)
{
    return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

// Frame: 0xB5 0x62, class, id, length (2 bytes), payload, then a checksum over class..payload.
static void send_ubx(uint8_t messageClass, uint8_t id, const uint8_t *payload, uint16_t length)
{
    uint8_t header[6] = {0xB5, 0x62, messageClass, id, (uint8_t)(length & 0xFF), (uint8_t)(length >> 8)};
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++)
    {
        a += header[i];
        b += a;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        a += payload[i];
        b += a;
    }
    uint8_t checksum[2] = {a, b};
    uart_write_blocking(port, header, sizeof(header));
    uart_write_blocking(port, payload, length);
    uart_write_blocking(port, checksum, sizeof(checksum));
}

// UBX-CFG-VALSET payload: version 0, RAM layer only, then key-value pairs added with add_key.
struct ValSet
{
    uint8_t bytes[64] = {0, 0x01, 0, 0};
    uint16_t length = 4;

    void add_key(uint32_t key, uint32_t value, int size)
    {
        put_u32(&bytes[length], key);
        for (int i = 0; i < size; i++)
        {
            bytes[length + 4 + i] = (value >> (8 * i)) & 0xFF;
        }
        length += 4 + size;
    }
};

// The receiver's UART at GPS_BAUD, UBX in and out, NMEA out off.
static void set_port()
{
    uint8_t prt[20] = {0};
    prt[0] = 1;                         // UART1
    put_u32(&prt[4], 0x000008D0);       // 8 data bits, no parity, 1 stop bit
    put_u32(&prt[8], GPS_BAUD);
    put_u16(&prt[12], 0x0003);          // in: UBX and NMEA
    put_u16(&prt[14], 0x0001);          // out: UBX only
    send_ubx(CLASS_CFG, ID_CFG_PRT, prt, sizeof(prt));

    ValSet valset;
    valset.add_key(KEY_UART1_BAUDRATE, GPS_BAUD, 4);
    valset.add_key(KEY_UART1OUTPROT_UBX, 1, 1);
    valset.add_key(KEY_UART1OUTPROT_NMEA, 0, 1);
    send_ubx(CLASS_CFG, ID_CFG_VALSET, valset.bytes, valset.length);
}

// NAV-PVT on every solution, a solution every rateMs, airborne dynamic model.
static void set_messages(uint rateMs)
{
    uint8_t msg[3] = {CLASS_NAV, ID_NAV_PVT, 1};
    send_ubx(CLASS_CFG, ID_CFG_MSG, msg, sizeof(msg));
    uint8_t rate[6];
    put_u16(&rate[0], rateMs);
    put_u16(&rate[2], 1);               // a solution per measurement
    put_u16(&rate[4], 1);               // aligned to GPS time
    send_ubx(CLASS_CFG, ID_CFG_RATE, rate, sizeof(rate));
    uint8_t nav5[36] = {0};
    put_u16(&nav5[0], 0x0001);          // only the dynamic model is changed
    nav5[2] = DYNMODEL_AIRBORNE_1G;
    send_ubx(CLASS_CFG, ID_CFG_NAV5, nav5, sizeof(nav5));

    ValSet valset;
    valset.add_key(KEY_MSGOUT_NAV_PVT_UART1, 1, 1);
    valset.add_key(KEY_RATE_MEAS, rateMs, 2);
    valset.add_key(KEY_NAVSPG_DYNMODEL, DYNMODEL_AIRBORNE_1G, 1);
    send_ubx(CLASS_CFG, ID_CFG_VALSET, valset.bytes, valset.length);
}

void gps_init(uart_inst_t *uart, uint txPin, uint rxPin, uint rateMs)
{
    port = uart;
    uart_init(port, 9600);
    gpio_set_function(txPin, UART_FUNCSEL_NUM(port, txPin));
    gpio_set_function(rxPin, UART_FUNCSEL_NUM(port, rxPin));

    // u-blox receivers start at 9600 (M8) or 38400 (M10) unless set otherwise, so ask at every likely rate.
    const uint bauds[] = {9600, 38400, 57600, 115200, 230400};
    for (uint baud : bauds)
    {
        uart_set_baudrate(port, baud);
        set_port();
        uart_tx_wait_blocking(port);
        sleep_ms(50);
    }
    uart_set_baudrate(port, GPS_BAUD);
    set_messages(rateMs);
    uart_tx_wait_blocking(port);

    // Only listen from here on: anything before was at the wrong baud rate.
    while (uart_is_readable(port))
    {
        uart_getc(port);
    }
    irq_set_exclusive_handler(UART_IRQ_NUM(port), on_uart_rx);
    irq_set_enabled(UART_IRQ_NUM(port), true);
    uart_set_irqs_enabled(port, true, false);
}

// Message being received: class, id, length, payload, checksum. Only messages up to NAV-PVT's size fit;
// longer ones are skipped.
static uint8_t frame[4 + NAV_PVT_LENGTH + 2];
static uint16_t frameBytes = 0;
static int syncBytes = 0;

// Adds a byte; returns true when it completes a message with a correct checksum.
static bool add_byte(uint8_t byte)
{
    if (syncBytes < 2)
    {
        if (byte == (syncBytes == 0 ? 0xB5 : 0x62))
        {
            syncBytes++;
        }
        else
        {
            syncBytes = byte == 0xB5 ? 1 : 0;
        }
        frameBytes = 0;
        return false;
    }
    frame[frameBytes++] = byte;
    if (frameBytes < 4)
    {
        return false;
    }
    uint16_t length = frame[2] | (frame[3] << 8);
    if (length > NAV_PVT_LENGTH)
    {
        syncBytes = 0;
        return false;
    }
    if (frameBytes < 4 + length + 2)
    {
        return false;
    }
    syncBytes = 0;
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < 4 + length; i++)
    {
        a += frame[i];
        b += a;
    }
    return a == frame[4 + length] && b == frame[5 + length];
}

bool gps_read(GpsFix &fix)
{
    bool newFix = false;
    while (rxTail != rxHead)
    {
        uint8_t byte = rxBuffer[rxTail % RX_BUFFER_SIZE];
        rxTail++;
        if (!add_byte(byte) || frame[0] != CLASS_NAV || frame[1] != ID_NAV_PVT ||
            (frame[2] | (frame[3] << 8)) != NAV_PVT_LENGTH)
        {
            continue;
        }
        const uint8_t *p = &frame[4];
        fix.fixType = p[20];
        fix.fixOk = (p[21] & 0x01) && !(p[78] & 0x01);     // gnssFixOK, and not invalidLlh
        fix.satellites = p[23];
        fix.longitude = (int32_t)get_u32(&p[24]);
        fix.latitude = (int32_t)get_u32(&p[28]);
        fix.altitude = (int32_t)get_u32(&p[36]) / 1000.0f;
        fix.horizontalAccuracy = get_u32(&p[40]) / 1000.0f;
        for (int axis = 0; axis < 3; axis++)
        {
            fix.velocity[axis] = (int32_t)get_u32(&p[48 + 4 * axis]) / 1000.0f;
        }
        fix.speedAccuracy = get_u32(&p[68]) / 1000.0f;
        newFix = true;
    }
    return newFix;
}

void gps_offset(int32_t latitude, int32_t longitude, int32_t originLatitude, int32_t originLongitude,
                float &north, float &east)
{
    const float metresPerUnit = 6371000.0f * 3.14159265f / 180.0f / 1e7f;    // per 1e-7 degree of latitude
    int64_t dLon = (int64_t)longitude - originLongitude;
    if (dLon > 1800000000)
    {
        dLon -= 3600000000LL;
    }
    else if (dLon < -1800000000)
    {
        dLon += 3600000000LL;
    }
    north = (float)(latitude - originLatitude) * metresPerUnit;
    east = (float)dLon * metresPerUnit * cosf(originLatitude * 1e-7f * 3.14159265f / 180.0f);
}
