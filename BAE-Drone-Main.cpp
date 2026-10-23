#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include <RF24.h>
#include "include/bno.h"
#include "include/pid.h"
#include "include/mixer.h"
#include "include/motors.h"
#include "include/attitude.h"
#include "include/filters.h"
#include "include/baro.h"
#include "include/altitude.h"
#include "include/gps.h"
#include "include/compass.h"
#include "include/position.h"

#define CE_PIN 20
#define CSN_PIN 17

// ESC signal pins, in mixer order (see include/mixer.h for where each motor sits and which way it spins).
const uint MOTOR_PINS[MOTOR_COUNT] = {6, 7, 8, 9};   // M1 rear right, M2 front right, M3 rear left, M4 front left
#define ESC_RATE_HZ 400            // pulses per second; use 50 for ESCs made for planes or cars
#define MOTOR_STOP_US 1000         // pulse while disarmed: motors off
#define MOTOR_IDLE_US 1080         // lowest pulse while armed: motors spin slowly; raise it if a motor stutters
#define MOTOR_MAX_US 2000          // full throttle

#define LOOP_PERIOD_US 2500        // 400 Hz control loop, matching the ESC pulse rate
#define LOOP_RATE_HZ (1000000.0f / LOOP_PERIOD_US)
#define MAX_PID_DT_S 0.05f         // cap on the PID time step after missed IMU reads
#define WATCHDOG_TIMEOUT_MS 100    // reboot (which leaves the drone disarmed) if the loop stalls this long
#define IMU_TIMEOUT_US 50000       // IMU counts as failed after 50 ms without a new gyro reading

// Gyro calibration, filtering and the tilt estimate (the BNO055 runs in raw mode; see main()).
#define GYRO_CAL_SAMPLES 400           // readings averaged for the gyro bias at power-up (1 s)
#define GYRO_CAL_MAX_SPREAD_DPS 3.0f   // more spread than this while calibrating means the drone moved
#define GYRO_CAL_MAX_BIAS_DPS 5.0f     // so does an average further from zero than this
#define GYRO_LPF_HZ 100.0f         // gyro low-pass filter; 0 turns it off
#define GYRO_NOTCH_HZ 0.0f         // notch filter at a motor vibration frequency, once logs show one; 0 is off
#define GYRO_NOTCH_Q 2.0f          // notch width: higher is narrower
#define TILT_KP 1.0f               // how fast the accelerometer corrects the tilt estimate, per second
#define ACCEL_TRUST_BAND 0.2f      // only use the accelerometer while it reads within 20% of 1 g

// Radio failsafe, timed from the last valid packet.
#define FAILSAFE_LEVEL_US 250000   // after 0.25 s: level out and descend
#define FAILSAFE_DISARM_US 2000000 // after 2 s: disarm

// Transmitter stick scale: assumed to be 0-1023 analog readings centred at 512.
// Change these to match what the transmitter actually sends.
#define STICK_CENTER 512
#define STICK_HALF_RANGE 512
#define THROTTLE_MIN 0
#define THROTTLE_MAX 1023
#define THROTTLE_IDLE 50           // at or below this the throttle counts as low
#define FAILSAFE_THROTTLE 400      // descent throttle during failsafe; set it just below hover

#define ARM_HOLD_US 1000000        // how long an arm or disarm stick command must be held
#define ARM_MAX_TILT_DEG 25.0f     // refuse to arm unless roughly level
#define CRASH_TILT_DEG 60.0f       // disarm past this tilt while armed

// Flight mode. Self-level (0): roll/pitch sticks set an angle, and the drone levels itself when they
// are centred. Acro (1, for tuning the rate loop on a test rig): roll/pitch sticks set a rotation rate,
// and the drone holds whatever angle it is at. Failsafe always self-levels.
#define ACRO_MODE 0

#define STICK_DEADBAND 0.03f       // fraction of stick travel around centre that counts as centred
#define STICK_EXPO 0.3f            // 0 is linear; higher is softer around centre (rate sticks only)
#define MAX_ANGLE_DEG 30.0f        // self-level: roll/pitch angle at full stick
#define MAX_ROLL_PITCH_RATE_DPS 200.0f  // acro: roll/pitch rate at full stick; also caps self-level
#define MAX_YAW_RATE_DPS 150.0f    // yaw rate at full stick (yaw is always rate-controlled)
#define TILT_COMP_MAX 1.3f         // self-level: most the throttle is raised to make up for tilt (about 40 deg)

// Self-level outer loop: deg/s of rotation rate asked for per degree of angle error.
#define ANGLE_KP 4.0f

// Rate PIDs (inner loop): microseconds of ESC pulse per deg/s of rate error.
// These are starting points for the 1000 us throttle span; tune them on a test rig.
#define RATE_ROLL_PITCH_KP 0.7f
#define RATE_ROLL_PITCH_KI 10.0f
#define RATE_ROLL_PITCH_KD 0.01f
#define RATE_YAW_KP 1.0f
#define RATE_YAW_KI 5.0f
#define RATE_YAW_KD 0.0f
#define RATE_INTEGRAL_LIMIT 100.0f
#define DTERM_CUTOFF_HZ 25.0f
#define PID_OUTPUT_LIMIT 250.0f    // most one axis can add to or take from a motor, in microseconds

// Heading hold (self-level only): once the yaw stick is centred and the drone has stopped turning, it holds
// that heading, so a gust or a yaw torque change doesn't leave it pointing somewhere else. The heading comes
// from the gyro alone, so it drifts slowly, but only the change since the stick was centred matters.
// 0 turns it off.
#define HEADING_HOLD 1
#define HEADING_KP 1.5f                 // deg/s of yaw rate asked for per degree of heading error
#define HEADING_CAPTURE_RATE_DPS 10.0f  // the hold starts once the drone has turned slower than this...
#define HEADING_SETTLE_US 300000        // ...for this long, so any overshoot after a fast turn isn't pulled back

// Altitude hold needs a BMP388 or BMP390 barometer on the IMU's I2C bus; without one the drone flies as
// before and the switch does nothing. Turn the switch on while hovering: the throttle stick's position then
// becomes the hover point, and moving the stick up or down from there climbs or descends. Turning the
// switch off gives manual throttle back at the stick's position. Pulling the stick to idle, losing the
// radio link or losing the barometer also turns altitude hold off, and it then stays off until the switch
// is turned off and on again, so it never comes back on by itself.
#define BARO_POLL_LOOPS 4               // check for a new barometer reading every 4th loop (100 Hz; it measures at 50 Hz)
#define BARO_TIMEOUT_US 200000          // barometer counts as failed after 0.2 s without a new reading
#define ALT_EST_TIME_CONSTANT_S 3.0f    // how long the altitude estimate rides on the accelerometer
#define ALT_EST_MAX_ACCEL_MS2 5.0f      // vertical acceleration beyond half a g is taken as a bad reading
#define ALT_EST_MAX_OFFSET_MS2 2.0f     // largest accelerometer offset the estimate will learn
#define ALT_KP 1.0f                     // m/s of climb rate asked for per metre of altitude error
#define MAX_CLIMB_RATE_MS 1.0f          // climb or descent rate with the stick at the end of its travel
#define ALT_STICK_DEADBAND 0.1f         // throttle stick travel either side of the hover point that still holds
#define CLIMB_KP 0.15f                  // throttle (0..1 of the range above idle) per m/s of climb rate error
#define CLIMB_KI 0.3f
#define CLIMB_INTEGRAL_LIMIT 0.2f
#define CLIMB_OUTPUT_LIMIT 0.3f         // most altitude hold moves the throttle away from the hover throttle
#define GRAVITY_MS2 9.80665f

// Compass: the BNO055's magnetometer. Position hold needs it, to know which way to tilt to reach the hold
// point. Motor currents and power wires disturb it, so keep the battery leads away from the BNO055 and
// check it on the bench (see IMU_DEBUG_PRINT). 0 turns it off, and with it position hold.
#define USE_COMPASS 1
// Hard-iron offsets (uT, BNO055 axes). With IMU_DEBUG_PRINT on and the motors off, turn the drone slowly
// through every orientation (all the way round level, nose up and down, on each side and upside down);
// the debug output shows the offsets found so far. Copy them here and reflash.
#define MAG_OFFSET_X 0.0f
#define MAG_OFFSET_Y 0.0f
#define MAG_OFFSET_Z 0.0f
#define MAG_DECLINATION_DEG 0.0f        // true north minus magnetic north, east positive; look it up for where you fly
#define COMPASS_KP 0.5f                 // how fast the compass corrects the gyro heading, per second
#define COMPASS_MIN_FIELD_UT 15.0f      // Earth's field is 25-65 uT; readings outside this range aren't used
#define COMPASS_MAX_FIELD_UT 100.0f
#define COMPASS_MAX_TILT_DEG 45.0f      // nor are readings taken more tilted than this
#define COMPASS_TIMEOUT_US 500000       // position hold stops after 0.5 s without a usable compass reading

// Position hold needs a u-blox GPS (M8, M9 or M10) on UART0: GP0 to the GPS's RX, GP1 to its TX. It also
// holds altitude, so it needs the barometer too. Turn its switch on while hovering: the drone holds its
// position, and the roll and pitch sticks fly it at up to MAX_POS_SPEED_MS relative to the way it points
// (the pitch stick direction that tips the nose down in self-level flies it forward). Centre them and it
// stops and holds there. The throttle stick works as in altitude hold. Losing the GPS fix or the compass
// drops it back to altitude hold, and position hold then stays off until its switch is turned off and on again.
#define GPS_UART uart0
#define GPS_TX_PIN 0
#define GPS_RX_PIN 1
#define GPS_RATE_MS 200                 // 5 fixes a second
#define GPS_TIMEOUT_US 600000           // GPS counts as lost after 0.6 s without a good fix
#define GPS_MIN_SATELLITES 6
#define GPS_MAX_POSITION_ERROR_M 3.0f   // the receiver's own accuracy estimates must be at least this good
#define GPS_MAX_SPEED_ERROR_MS 1.0f
#define GPS_DELAY_S 0.1f                // a fix describes where the drone was this long before it arrives
#define POS_EST_POSITION_TIME_CONSTANT_S 2.0f
#define POS_EST_VELOCITY_TIME_CONSTANT_S 1.0f
#define POS_EST_MAX_ACCEL_MS2 6.0f      // horizontal acceleration beyond this is taken as a bad reading
#define POS_EST_MAX_OFFSET_MS2 2.0f
#define POS_KP 0.5f                     // m/s of speed asked for per metre from the hold point
#define MAX_POS_SPEED_MS 3.0f           // speed asked for with a stick at the end of its travel (air drag keeps it a little under)
#define POS_CAPTURE_SPEED_MS 0.3f       // after the sticks are centred, the hold point is set once slower than this
#define VEL_KP 1.5f                     // m/s^2 of acceleration asked for per m/s of speed error
#define VEL_KI 0.5f                     // the integral leans into a steady wind; it's kept through the flight, and doesn't change during stick flight
#define VEL_INTEGRAL_LIMIT 2.0f
#define MAX_POS_TILT_DEG 20.0f          // most position hold tilts the drone, each of roll and pitch

// How the BNO055 is mounted: which of its axes (0 = X, 1 = Y, 2 = Z, as marked in its datasheet or on
// the breakout board) points to the drone's front, right side and bottom, and with which sign. The
// defaults are for the board lying flat, components up, with its Y arrow forward and X arrow to the right.
// Both the rates and the angles come from this, so it decides which way positive roll (right side down),
// pitch (nose up) and yaw (clockwise seen from above) go. Check it on the bench before flying
// (see IMU_DEBUG_PRINT): a wrong setting makes the drone flip.
#define FORWARD_AXIS 1
#define FORWARD_SIGN 1.0f
#define RIGHT_AXIS 0
#define RIGHT_SIGN 1.0f
#define DOWN_AXIS 2
#define DOWN_SIGN -1.0f

// Level trim. With the drone sitting on a level surface, add the roll and pitch it reads (see
// IMU_DEBUG_PRINT) to these, so self-level holds the frame level rather than the sensor board.
#define ROLL_TRIM_DEG 0.0f
#define PITCH_TRIM_DEG 0.0f

#define RAD_TO_DEG 57.2957795f

// While disarmed, print the angles, rates and total tilt 5 times a second over USB serial. Sitting level,
// roll and pitch must read near 0 (near 180 means the mounting settings are upside down). Lowering the
// right side must make the roll angle and roll rate positive; raising the nose must make the pitch angle
// and pitch rate positive; turning clockwise seen from above must make the yaw rate positive.
// It also prints how many new gyro readings arrive per second: that should be about 400, the loop rate.
// With a barometer it prints the altitude relative to power-up and the climb rate: lifting the drone by
// 1 m should raise the altitude by about 1, with the climb rate positive while it rises. A second line shows
// the heading: facing north it should read near 0, and turning clockwise should raise it (east is 90). It
// should stay steady when the motors run (props off); if it moves, the compass is too close to the power
// wiring. The line also shows the magnetometer offsets found so far (see MAG_OFFSET_X) and the GPS status.
// Set to 0 once the signs are confirmed.
#define IMU_DEBUG_PRINT 1

RF24 radio(CE_PIN, CSN_PIN);

// Stick values as used by the controller.
struct JoystickData
{
    int throttle;
    int yaw;
    int pitch;
    int roll;
    bool altitudeHold;      // the altitude hold switch
    bool positionHold;      // the position hold switch (also holds altitude)
};

// Radio payload from the transmitter. The transmitter must send exactly this layout (little-endian)
// with the same payload size, add 1 to sequence on every packet, and set checksum to the XOR of
// all the bytes before it.
struct __attribute__((packed)) ControlPacket
{
    uint8_t sequence;
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t switches;       // SWITCH_ bits below
    uint8_t checksum;
};
#define SWITCH_ALTITUDE_HOLD 0x01
#define SWITCH_POSITION_HOLD 0x02

const JoystickData NEUTRAL_STICKS = {THROTTLE_MIN, STICK_CENTER, STICK_CENTER, STICK_CENTER, false, false};

Pid rollRatePid(RATE_ROLL_PITCH_KP, RATE_ROLL_PITCH_KI, RATE_ROLL_PITCH_KD, PID_OUTPUT_LIMIT, RATE_INTEGRAL_LIMIT, DTERM_CUTOFF_HZ);
Pid pitchRatePid(RATE_ROLL_PITCH_KP, RATE_ROLL_PITCH_KI, RATE_ROLL_PITCH_KD, PID_OUTPUT_LIMIT, RATE_INTEGRAL_LIMIT, DTERM_CUTOFF_HZ);
Pid yawRatePid(RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, PID_OUTPUT_LIMIT, RATE_INTEGRAL_LIMIT, DTERM_CUTOFF_HZ);
Pid *const ratePids[] = {&rollRatePid, &pitchRatePid, &yawRatePid};
Pid climbRatePid(CLIMB_KP, CLIMB_KI, 0, CLIMB_OUTPUT_LIMIT, CLIMB_INTEGRAL_LIMIT, 0);
// Position hold's velocity loops (north, east): m/s^2 of acceleration out, up to what MAX_POS_TILT_DEG gives.
Pid northVelPid(VEL_KP, VEL_KI, 0, GRAVITY_MS2 * tanf(MAX_POS_TILT_DEG / RAD_TO_DEG), VEL_INTEGRAL_LIMIT, 0);
Pid eastVelPid(VEL_KP, VEL_KI, 0, GRAVITY_MS2 * tanf(MAX_POS_TILT_DEG / RAD_TO_DEG), VEL_INTEGRAL_LIMIT, 0);

// Repeats the error forever so it is still visible once USB serial connects.
static void halt(const char *msg, int code)
{
    while (true)
    {
        printf("%s (code %d)\n", msg, code);
        sleep_ms(1000);
    }
}

static float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

// Maps a raw stick reading to -1..1 around the centre.
static float stick_to_unit(int raw)
{
    return clamp((raw - STICK_CENTER) / (float)STICK_HALF_RANGE, 1.0f);
}

// Like stick_to_unit, plus a centre deadband and expo (softer around centre, full range kept at the ends).
static float shape_stick(int raw, float expo)
{
    float unit = stick_to_unit(raw);
    if (fabsf(unit) < STICK_DEADBAND)
    {
        return 0;
    }
    // Rescale so the output starts from 0 at the edge of the deadband.
    unit = (unit - copysignf(STICK_DEADBAND, unit)) / (1.0f - STICK_DEADBAND);
    return unit * (1.0f - expo) + unit * unit * unit * expo;
}

// Maps the throttle stick to 0..1 of the motor range above idle.
static float throttle_fraction(int throttle)
{
    float fraction = (throttle - THROTTLE_IDLE) / (float)(THROTTLE_MAX - THROTTLE_IDLE);
    return fraction < 0 ? 0 : (fraction > 1 ? 1 : fraction);
}

// The throttle stick in altitude hold: -1..1 for full descent..full climb, measured from the hover point
// (where the stick was when altitude hold came on) to each end of the stick's travel, with a deadband.
static float climb_stick(int throttle, int hoverStick)
{
    float span = throttle >= hoverStick ? THROTTLE_MAX - hoverStick : hoverStick - THROTTLE_IDLE;
    float unit = clamp((throttle - hoverStick) / fmaxf(span, 1.0f), 1.0f);
    if (fabsf(unit) < ALT_STICK_DEADBAND)
    {
        return 0;
    }
    return (unit - copysignf(ALT_STICK_DEADBAND, unit)) / (1.0f - ALT_STICK_DEADBAND);
}

// Wraps an angle in degrees to -180..180.
static float wrap_180(float degrees)
{
    return degrees - 360.0f * floorf((degrees + 180.0f) / 360.0f);
}

static int16_t read_int16(const uint8_t *bytes)
{
    return (int16_t)((bytes[1] << 8) | bytes[0]);
}

// Forward, right and down must be three different BNO055 axes forming a right-handed set,
// or no real mounting matches them.
constexpr bool mounting_is_valid()
{
    float forward[3] = {}, right[3] = {}, down[3] = {};
    forward[FORWARD_AXIS] = FORWARD_SIGN;
    right[RIGHT_AXIS] = RIGHT_SIGN;
    down[DOWN_AXIS] = DOWN_SIGN;
    return forward[1] * right[2] - forward[2] * right[1] == down[0] &&
           forward[2] * right[0] - forward[0] * right[2] == down[1] &&
           forward[0] * right[1] - forward[1] * right[0] == down[2];
}
static_assert(mounting_is_valid(), "FORWARD/RIGHT/DOWN settings are mirrored: if one direction looked reversed on the "
                                   "bench, the board is turned relative to them and two of the settings need to change");

// Converts a vector from the BNO055's axes to the drone's: forward, right, down.
static void sensor_to_body(const float sensor[3], float body[3])
{
    body[0] = FORWARD_SIGN * sensor[FORWARD_AXIS];
    body[1] = RIGHT_SIGN * sensor[RIGHT_AXIS];
    body[2] = DOWN_SIGN * sensor[DOWN_AXIS];
}

static uint8_t xor_checksum(const uint8_t *bytes, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
    {
        checksum ^= bytes[i];
    }
    return checksum;
}

// Reads one payload into sticks. Returns false, leaving sticks alone, if the checksum is bad or the
// sequence number repeats, since a repeated packet doesn't show the transmitter is still running.
static bool read_packet(JoystickData &sticks, int &lastSequence)
{
    ControlPacket packet;
    radio.read(&packet, sizeof(packet));

    if (xor_checksum((const uint8_t *)&packet, sizeof(packet) - 1) != packet.checksum || packet.sequence == lastSequence)
    {
        return false;
    }
    lastSequence = packet.sequence;
    sticks = {packet.throttle, packet.yaw, packet.pitch, packet.roll, (packet.switches & SWITCH_ALTITUDE_HOLD) != 0,
              (packet.switches & SWITCH_POSITION_HOLD) != 0};
    return true;
}

// Returns why arming isn't allowed right now, or nullptr if it is.
static const char *arming_refusal(bool imuOk, bool gyroCalibrated, bool accelOk, float tiltAngle)
{
    if (!imuOk)
    {
        return "IMU not responding";
    }
    if (!gyroCalibrated)
    {
        return "gyro still calibrating, keep the drone still for a second";
    }
    if (!accelOk)
    {
        return "accelerometer doesn't read 1 g";
    }
    if (tiltAngle > ARM_MAX_TILT_DEG)
    {
        return "not level";
    }
    return nullptr;
}

// Gyro bias calibration: averages GYRO_CAL_SAMPLES readings taken while the drone sits still.
struct GyroCalibration
{
    float sum[3] = {0, 0, 0};
    float lowest[3], highest[3];
    int count = 0;
};

// Adds one reading (deg/s, BNO055 axes). Returns true, with bias filled in, once enough readings have
// been averaged. Starts over if the readings spread too much or average too far from zero, either of
// which means the drone moved.
static bool calibrate_gyro(GyroCalibration &cal, const float gyro[3], float bias[3])
{
    for (int axis = 0; axis < 3; axis++)
    {
        if (cal.count == 0 || gyro[axis] < cal.lowest[axis])
        {
            cal.lowest[axis] = gyro[axis];
        }
        if (cal.count == 0 || gyro[axis] > cal.highest[axis])
        {
            cal.highest[axis] = gyro[axis];
        }
        cal.sum[axis] += gyro[axis];
    }
    cal.count++;

    for (int axis = 0; axis < 3; axis++)
    {
        if (cal.highest[axis] - cal.lowest[axis] > GYRO_CAL_MAX_SPREAD_DPS ||
            (cal.count == GYRO_CAL_SAMPLES && fabsf(cal.sum[axis] / cal.count) > GYRO_CAL_MAX_BIAS_DPS))
        {
            cal = GyroCalibration();
            return false;
        }
    }
    if (cal.count < GYRO_CAL_SAMPLES)
    {
        return false;
    }
    for (int axis = 0; axis < 3; axis++)
    {
        bias[axis] = cal.sum[axis] / cal.count;
    }
    return true;
}


int main()
{
    // Send the stop pulse straight away: ESCs need to see it at power-up before they will arm.
    motors_init(MOTOR_PINS, ESC_RATE_HZ, MOTOR_STOP_US);

    stdio_init_all();

    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // The BNO055 needs about 650 ms after power-up before it answers on I2C.
    sleep_ms(700);

    bno055_t bno;

    bno.dev_addr = 0x28;
    bno.bus_write = bno055_bus_write;
    bno.bus_read = bno055_bus_read;
    bno.delay_msec = bno055_delay_msek;

    s8 com_rslt = bno055_init(&bno);
    if (com_rslt != 0)
    {
        halt("BNO055 initialization failed", com_rslt);
    }
    // Raw sensor readings, no fusion: the BNO055's own fusion only updates at 100 Hz, so the tilt is
    // estimated here at the loop rate instead, and the heading here from the gyro and compass. A 116 Hz gyro
    // filter gives 1 kHz gyro output, and the 8 g accelerometer range leaves headroom for motor vibration.
    // The magnetometer measures 20 times a second, which is plenty for correcting the gyro heading. The
    // data reads below assume register page 0.
    com_rslt = bno055_set_gyro_bw(BNO055_GYRO_BW_116HZ);
    com_rslt += bno055_set_accel_range(BNO055_ACCEL_RANGE_8G);
    com_rslt += bno055_set_mag_data_output_rate(BNO055_MAG_DATA_OUTRATE_20HZ);
    com_rslt += bno055_set_mag_operation_mode(BNO055_MAG_OPERATION_MODE_ENHANCED_REGULAR);
    com_rslt += bno055_set_operation_mode(USE_COMPASS ? BNO055_OPERATION_MODE_AMG : BNO055_OPERATION_MODE_ACCGYRO);
    com_rslt += bno055_write_page_id(BNO055_PAGE_ZERO);
    if (com_rslt != 0)
    {
        printf("Failed to configure the BNO055, error: %d\n", com_rslt);
    }
    sleep_ms(1000);

    struct bno055_euler_float_t eulerData;
    bno055_convert_float_euler_hpr_deg(&eulerData);

    unsigned char accel_calib_status = 0;
    unsigned char gyro_calib_status = 0;
    unsigned char mag_calib_status = 0;
    unsigned char sys_calib_status = 0;
    JoystickData sticks = NEUTRAL_STICKS;

    bno055_get_accel_calib_stat(&accel_calib_status);
    bno055_get_mag_calib_stat(&mag_calib_status);
    bno055_get_gyro_calib_stat(&gyro_calib_status);
    bno055_get_sys_calib_stat(&sys_calib_status);

    bool baroPresent = baro_init(I2C_PORT);
    if (baroPresent)
    {
        printf("Barometer found: altitude hold available\n");
    }
    else
    {
        printf("No barometer found: altitude hold unavailable\n");
    }
    gps_init(GPS_UART, GPS_TX_PIN, GPS_RX_PIN, GPS_RATE_MS);

    if (!radio.begin())
    {
        halt("NRF24L01 failed to initialize", 0);
    }

    radio.setPayloadSize(sizeof(ControlPacket));
    radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
    radio.setPALevel(RF24_PA_HIGH);
    radio.startListening();

    bool armed = false;
    bool inFailsafe = false;
    uint64_t lastPacketUs = 0;
    int lastSequence = -1;
    uint64_t stickCommandStartUs = 0;            // when the current arm/disarm stick command began, 0 if none
    uint64_t lastImuReadUs = 0;                  // last successful read
    uint64_t lastImuUpdateUs = 0;                // last read with a new gyro value, 0 before the first
    int16_t lastGyroRaw[3] = {0, 0, 0};
    int gyroUpdates = 0;                         // new gyro readings since the last debug print
    GyroCalibration gyroCal;
    float gyroBias[3] = {0, 0, 0};
    bool gyroCalibrated = false;
    TiltEstimator tilt(TILT_KP, ACCEL_TRUST_BAND);
    Biquad rateLowpass[3], rateNotch[3];
    for (int axis = 0; axis < 3; axis++)
    {
        rateLowpass[axis] = Biquad::lowpass(GYRO_LPF_HZ, LOOP_RATE_HZ);
        rateNotch[axis] = Biquad::notch(GYRO_NOTCH_HZ, GYRO_NOTCH_Q, LOOP_RATE_HZ);
    }
    float rollAngle = 0, pitchAngle = 0;
    float tiltAngle = 0, tiltCos = 1;            // total tilt from level, whichever way
    float rollRate = 0, pitchRate = 0, yawRate = 0;
    float rollOutput = 0, pitchOutput = 0, yawOutput = 0;
    bool mixerSaturated = false;                 // from the previous loop; holds the PID integrals
    uint64_t lastPidUs = 0;                      // time of the last PID update, 0 while they are reset
    uint64_t lastDebugPrintUs = 0;
    uint32_t loopCount = 0;
    float heading = 0;                           // deg from north, clockwise; from the gyro, corrected by the compass
    float headingTarget = 0;
    bool headingHeld = false;
    uint64_t headingSettleStartUs = 0;           // when the turn rate last dropped below the capture rate, 0 if not
    AltitudeEstimator altitude(ALT_EST_TIME_CONSTANT_S, ALT_EST_MAX_ACCEL_MS2, ALT_EST_MAX_OFFSET_MS2);
    uint64_t lastBaroUs = 0;                     // last new barometer reading
    float baroZero = 0;                          // altitude of the first reading, so altitudes start near 0
    bool altHold = false;
    bool altHoldReady = false;                   // the switch has been off since altitude hold last stopped
    bool lastAltHoldSwitch = false;
    float altTarget = 0, hoverThrottle = 0, climbOutput = 0;
    int hoverStick = 0;
    const float magOffsets[3] = {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z};
    CompassCalibration compassCal;               // readings seen while disarmed, for finding the offsets
    bool compassOk = false;                      // last reading usable
    bool headingFromCompass = false;             // heading has been set from the compass
    float compassHeading = 0, magField = 0;
    uint64_t lastCompassUs = 0;                  // last usable compass reading
    GpsFix gps = {};
    bool gpsReceiving = false;
    bool gpsOriginSet = false;
    int32_t originLatitude = 0, originLongitude = 0;    // where positions are measured from: the first good fix
    uint64_t lastGpsUs = 0;                      // last fix good enough to navigate by
    float lastFixVelocity[3] = {0, 0, 0};
    float gpsAccel[3] = {0, 0, 0};               // m/s^2 north, east, down, from the change in GPS velocity
    PositionEstimator north(POS_EST_POSITION_TIME_CONSTANT_S, POS_EST_VELOCITY_TIME_CONSTANT_S,
                            POS_EST_MAX_ACCEL_MS2, POS_EST_MAX_OFFSET_MS2, (int)(GPS_DELAY_S * LOOP_RATE_HZ));
    PositionEstimator east(POS_EST_POSITION_TIME_CONSTANT_S, POS_EST_VELOCITY_TIME_CONSTANT_S,
                           POS_EST_MAX_ACCEL_MS2, POS_EST_MAX_OFFSET_MS2, (int)(GPS_DELAY_S * LOOP_RATE_HZ));
    bool posHold = false;
    bool posHoldReady = false;                   // the switch has been off since position hold last stopped
    bool lastPosHoldSwitch = false;
    bool posHeld = false;                        // holding a point (rather than flying or braking)
    float posTarget[2] = {0, 0};                 // m north and east of the origin
    float posRollSetpoint = 0, posPitchSetpoint = 0;

    if (watchdog_enable_caused_reboot())
    {
        printf("Rebooted by the watchdog: the control loop stalled\n");
    }
    printf("Ready, disarmed\n");

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    absolute_time_t nextLoop = get_absolute_time();

    while (true)
    {
        watchdog_update();
        uint64_t now = time_us_64();

        // The receive FIFO holds up to 3 payloads; keep the newest valid one.
        for (int i = 0; i < 3 && radio.available(); i++)
        {
            if (read_packet(sticks, lastSequence))
            {
                lastPacketUs = now;
            }
        }
        uint64_t linkAge = lastPacketUs == 0 ? UINT64_MAX : now - lastPacketUs;
        bool linkOk = linkAge < FAILSAFE_LEVEL_US;

        // One read covers the accelerometer (0x08-0x0D, 100 LSB per m/s^2, pointing up at rest), the
        // magnetometer (0x0E-0x13, 16 LSB per uT; zero with USE_COMPASS off) and the gyro (0x14-0x19, 16 LSB per deg/s).
        uint8_t imuData[18] = {0};
        com_rslt = bno055_bus_read(bno.dev_addr, 0x08, imuData, 18);
        bool freshImu = com_rslt == 0;

        if (freshImu)
        {
            float accelSensor[3], gyroSensor[3], magSensor[3];
            bool gyroChanged = false;
            for (int axis = 0; axis < 3; axis++)
            {
                int16_t gyroRaw = read_int16(&imuData[12 + 2 * axis]);
                gyroChanged = gyroChanged || gyroRaw != lastGyroRaw[axis];
                lastGyroRaw[axis] = gyroRaw;
                gyroSensor[axis] = gyroRaw / 16.0f;
                accelSensor[axis] = read_int16(&imuData[2 * axis]) / 100.0f;
                magSensor[axis] = read_int16(&imuData[6 + 2 * axis]) / 16.0f;
            }
            if (!armed)
            {
                compassCal.add(magSensor);
            }
            for (int axis = 0; axis < 3; axis++)
            {
                magSensor[axis] -= magOffsets[axis];
            }
            // Noise changes the gyro reading every sample, so one that stops changing has stopped updating.
            if (gyroChanged)
            {
                lastImuUpdateUs = now;
                gyroUpdates++;
            }

            if (!gyroCalibrated && calibrate_gyro(gyroCal, gyroSensor, gyroBias))
            {
                gyroCalibrated = true;
                printf("Gyro calibrated, bias %.2f %.2f %.2f deg/s\n", gyroBias[0], gyroBias[1], gyroBias[2]);
            }
            for (int axis = 0; axis < 3; axis++)
            {
                gyroSensor[axis] -= gyroBias[axis];
            }

            float rates[3], accel[3], mag[3];
            sensor_to_body(gyroSensor, rates);
            sensor_to_body(accelSensor, accel);
            sensor_to_body(magSensor, mag);

            // The PIDs get filtered rates; the tilt estimate uses the unfiltered ones, which lag less.
            rollRate = rateNotch[0].apply(rateLowpass[0].apply(rates[0]));
            pitchRate = rateNotch[1].apply(rateLowpass[1].apply(rates[1]));
            yawRate = rateNotch[2].apply(rateLowpass[2].apply(rates[2]));

            if (!tilt.isReady())
            {
                tilt.reset(accel);
            }
            else
            {
                float dt = (now - lastImuReadUs) / 1e6f;
                dt = dt > MAX_PID_DT_S ? MAX_PID_DT_S : dt;

                // Besides gravity, the accelerometer feels the drone speeding up or slowing down, which would
                // pull the tilt estimate off while it accelerates. With GPS, take out the acceleration it measures.
                float gravityAccel[3] = {accel[0], accel[1], accel[2]};
                if (now - lastGpsUs < GPS_TIMEOUT_US)
                {
                    float linear[3];
                    ned_to_body(gpsAccel, tilt.up(), heading, linear);
                    for (int axis = 0; axis < 3; axis++)
                    {
                        gravityAccel[axis] -= linear[axis];
                    }
                }
                tilt.update(rates, gravityAccel, dt);

                // Turn rate about the vertical (positive clockwise seen from above) for heading hold, and
                // vertical acceleration with gravity taken out for the altitude estimate.
                const float *up = tilt.up();
                heading = wrap_180(heading - (rates[0] * up[0] + rates[1] * up[1] + rates[2] * up[2]) * dt);
                if (altitude.isReady())
                {
                    altitude.update(accel[0] * up[0] + accel[1] * up[1] + accel[2] * up[2] - GRAVITY_MS2, dt,
                                    now - lastBaroUs < BARO_TIMEOUT_US);
                }

                // The compass pulls the gyro heading toward it slowly, so its noise and brief disturbances
                // average out; the very first reading sets the heading outright.
                magField = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
                compassOk = USE_COMPASS && magField > COMPASS_MIN_FIELD_UT && magField < COMPASS_MAX_FIELD_UT &&
                            -up[2] > cosf(COMPASS_MAX_TILT_DEG / RAD_TO_DEG);
                if (compassOk)
                {
                    compassHeading = wrap_180(compass_heading(mag, up) + MAG_DECLINATION_DEG);
                    heading = headingFromCompass ? wrap_180(heading + wrap_180(compassHeading - heading) * COMPASS_KP * dt)
                                                 : compassHeading;
                    headingFromCompass = true;
                    lastCompassUs = now;
                }

                // Horizontal acceleration (the accelerometer turned to north, east, down; gravity only has a
                // down part) for the position estimate.
                if (north.isReady())
                {
                    float accelNed[3];
                    body_to_ned(accel, up, heading, accelNed);
                    bool gpsFresh = now - lastGpsUs < GPS_TIMEOUT_US;
                    north.update(accelNed[0], dt, gpsFresh);
                    east.update(accelNed[1], dt, gpsFresh);
                }
            }
            lastImuReadUs = now;

            const float *up = tilt.up();
            rollAngle = atan2f(-up[1], -up[2]) * RAD_TO_DEG - ROLL_TRIM_DEG;
            pitchAngle = atan2f(up[0], sqrtf(up[1] * up[1] + up[2] * up[2])) * RAD_TO_DEG - PITCH_TRIM_DEG;
            tiltCos = fminf(fmaxf(-up[2], -1.0f), 1.0f);
            tiltAngle = acosf(tiltCos) * RAD_TO_DEG;
        }
        bool imuOk = tilt.isReady() && lastImuUpdateUs != 0 && now - lastImuUpdateUs < IMU_TIMEOUT_US;

        if (baroPresent && loopCount % BARO_POLL_LOOPS == 0)
        {
            float pressure;
            if (baro_read(pressure) == 1)
            {
                float baroAltitude = pressure_to_altitude(pressure);
                if (!altitude.isReady())
                {
                    baroZero = baroAltitude;
                    altitude.reset(0);
                }
                else
                {
                    altitude.setBaro(baroAltitude - baroZero);
                }
                lastBaroUs = now;
            }
        }
        bool baroOk = altitude.isReady() && now - lastBaroUs < BARO_TIMEOUT_US;

        GpsFix fix;
        if (gps_read(fix))
        {
            if (!gpsReceiving)
            {
                gpsReceiving = true;
                printf("GPS connected\n");
            }
            gps = fix;
            bool goodFix = fix.fixType >= 3 && fix.fixOk && fix.satellites >= GPS_MIN_SATELLITES &&
                           fix.horizontalAccuracy <= GPS_MAX_POSITION_ERROR_M && fix.speedAccuracy <= GPS_MAX_SPEED_ERROR_MS;
            if (goodFix)
            {
                if (!gpsOriginSet)
                {
                    gpsOriginSet = true;
                    originLatitude = fix.latitude;
                    originLongitude = fix.longitude;
                    printf("GPS fix: %d satellites, within %.1f m\n", fix.satellites, fix.horizontalAccuracy);
                }
                float fixNorth, fixEast;
                gps_offset(fix.latitude, fix.longitude, originLatitude, originLongitude, fixNorth, fixEast);
                bool lastFixRecent = lastGpsUs != 0 && now - lastGpsUs < GPS_TIMEOUT_US;
                float fixInterval = (now - lastGpsUs) / 1e6f;
                for (int axis = 0; axis < 3; axis++)
                {
                    gpsAccel[axis] = lastFixRecent ? (fix.velocity[axis] - lastFixVelocity[axis]) / fixInterval : 0;
                    lastFixVelocity[axis] = fix.velocity[axis];
                }
                // After a gap the estimate may have drifted far, so start it again from the fix.
                if (!north.isReady() || now - lastGpsUs >= GPS_TIMEOUT_US)
                {
                    north.reset(fixNorth, fix.velocity[0]);
                    east.reset(fixEast, fix.velocity[1]);
                }
                else
                {
                    north.setGps(fixNorth, fix.velocity[0]);
                    east.setGps(fixEast, fix.velocity[1]);
                }
                lastGpsUs = now;
            }
        }
        bool gpsOk = north.isReady() && lastGpsUs != 0 && now - lastGpsUs < GPS_TIMEOUT_US;
        bool compassRecent = headingFromCompass && now - lastCompassUs < COMPASS_TIMEOUT_US;

        // Stick commands, with throttle low and a live link: hold full right yaw to arm, full left yaw to disarm.
        float yawStick = stick_to_unit(sticks.yaw);
        bool throttleLow = linkOk && sticks.throttle <= THROTTLE_IDLE;
        bool stickCommand = throttleLow && (armed ? yawStick < -0.9f : yawStick > 0.9f);
        if (!stickCommand)
        {
            stickCommandStartUs = 0;
        }
        else if (stickCommandStartUs == 0)
        {
            stickCommandStartUs = now;
        }
        else if (now - stickCommandStartUs >= ARM_HOLD_US)
        {
            stickCommandStartUs = 0;
            if (armed)
            {
                armed = false;
                printf("Disarmed\n");
            }
            else
            {
                // Sitting still before arming, the accelerometer must read about 1 g.
                const char *refusal = arming_refusal(imuOk, gyroCalibrated, tilt.accelInUse(), tiltAngle);
                if (refusal)
                {
                    printf("Arming refused: %s\n", refusal);
                }
                else
                {
                    armed = true;
                    printf("Armed\n");
                }
            }
        }

        if (armed)
        {
            const char *reason = nullptr;
            if (!imuOk)
            {
                reason = "IMU not updating";
            }
            else if (tiltAngle > CRASH_TILT_DEG)
            {
                reason = "tilted past the limit";
            }
            else if (linkAge >= FAILSAFE_DISARM_US)
            {
                reason = "radio link lost";
            }
            if (reason)
            {
                armed = false;
                printf("Disarmed: %s\n", reason);
            }
        }

        // Failsafe: level out and descend, never raising the throttle above the pilot's last value.
        JoystickData target = sticks;
        bool failsafe = armed && !linkOk;
        if (failsafe)
        {
            target = NEUTRAL_STICKS;
            target.throttle = sticks.throttle < FAILSAFE_THROTTLE ? sticks.throttle : FAILSAFE_THROTTLE;
        }
        if (failsafe && !inFailsafe)
        {
            printf("Failsafe: radio link lost, levelling and descending\n");
        }
        else if (!failsafe && inFailsafe && armed)
        {
            printf("Radio link back\n");
        }
        inFailsafe = failsafe;

        bool selfLevel = !ACRO_MODE || failsafe;
        // The rate PIDs only run in flight. On the ground (disarmed or throttle low) they stay reset.
        bool flying = armed && target.throttle > THROTTLE_IDLE;

        // Altitude hold, which position hold also turns on; see its settings at the top for how the switch
        // and the throttle stick work.
        bool altHoldSwitch = sticks.altitudeHold || sticks.positionHold;
        const char *altHoldBlocked = nullptr;
        if (!altHoldSwitch)
        {
            altHoldBlocked = "switch off";
        }
        else if (failsafe)
        {
            altHoldBlocked = "radio link lost";
        }
        else if (!flying)
        {
            altHoldBlocked = "disarmed or throttle at idle";
        }
        else if (!baroOk)
        {
            altHoldBlocked = baroPresent ? "barometer not working" : "no barometer";
        }
        if (altHoldBlocked)
        {
            if (altHold)
            {
                printf("Altitude hold off: %s\n", altHoldBlocked);
            }
            else if (altHoldSwitch && !lastAltHoldSwitch)
            {
                printf("Altitude hold can't start: %s\n", altHoldBlocked);
            }
            altHold = false;
            altHoldReady = !altHoldSwitch;
        }
        else if (!altHold && altHoldReady)
        {
            altHold = true;
            altTarget = altitude.altitude();
            hoverStick = target.throttle;
            hoverThrottle = throttle_fraction(target.throttle);
            climbRatePid.reset();
            climbOutput = 0;
            printf("Altitude hold on, hover throttle %.0f%%\n", hoverThrottle * 100);
        }
        lastAltHoldSwitch = altHoldSwitch;

        // Position hold; see its settings at the top.
        const char *posHoldBlocked = nullptr;
        if (!sticks.positionHold)
        {
            posHoldBlocked = "switch off";
        }
        else if (!altHold)
        {
            posHoldBlocked = "altitude hold is off";
        }
        else if (!selfLevel)
        {
            posHoldBlocked = "acro mode";
        }
        else if (!gpsOk)
        {
            posHoldBlocked = gpsReceiving ? "no good GPS fix" : "no GPS data";
        }
        else if (!compassRecent)
        {
            posHoldBlocked = USE_COMPASS ? "compass reading not usable" : "compass turned off";
        }
        if (posHoldBlocked)
        {
            if (posHold)
            {
                printf("Position hold off: %s\n", posHoldBlocked);
            }
            else if (sticks.positionHold && !lastPosHoldSwitch)
            {
                printf("Position hold can't start: %s\n", posHoldBlocked);
            }
            posHold = false;
            posHoldReady = !sticks.positionHold;
            posRollSetpoint = posPitchSetpoint = 0;
        }
        else if (!posHold && posHoldReady)
        {
            posHold = true;
            posHeld = false;
            printf("Position hold on\n");
        }
        lastPosHoldSwitch = sticks.positionHold;

        // Rotation rate targets. Self-level turns the angle error into a rate (the outer loop);
        // acro takes the rate straight from the sticks.
        float rollRateSetpoint, pitchRateSetpoint;
        if (selfLevel)
        {
            // In position hold the position controller sets the angles instead of the sticks.
            float rollAngleSetpoint = posHold ? posRollSetpoint : shape_stick(target.roll, 0) * MAX_ANGLE_DEG;
            float pitchAngleSetpoint = posHold ? posPitchSetpoint : shape_stick(target.pitch, 0) * MAX_ANGLE_DEG;
            rollRateSetpoint = clamp((rollAngleSetpoint - rollAngle) * ANGLE_KP, MAX_ROLL_PITCH_RATE_DPS);
            pitchRateSetpoint = clamp((pitchAngleSetpoint - pitchAngle) * ANGLE_KP, MAX_ROLL_PITCH_RATE_DPS);
        }
        else
        {
            rollRateSetpoint = shape_stick(target.roll, STICK_EXPO) * MAX_ROLL_PITCH_RATE_DPS;
            pitchRateSetpoint = shape_stick(target.pitch, STICK_EXPO) * MAX_ROLL_PITCH_RATE_DPS;
        }
        float yawRateSetpoint = shape_stick(target.yaw, STICK_EXPO) * MAX_YAW_RATE_DPS;

        // Heading hold locks on once the yaw stick is centred and the turn has stopped, and lets go when
        // the stick moves.
        if (!HEADING_HOLD || !selfLevel || !flying || yawRateSetpoint != 0)
        {
            headingHeld = false;
            headingSettleStartUs = 0;
        }
        else if (!headingHeld)
        {
            if (fabsf(yawRate) >= HEADING_CAPTURE_RATE_DPS)
            {
                headingSettleStartUs = 0;
            }
            else if (headingSettleStartUs == 0)
            {
                headingSettleStartUs = now;
            }
            else if (now - headingSettleStartUs >= HEADING_SETTLE_US)
            {
                headingHeld = true;
                headingTarget = heading;
            }
        }
        if (headingHeld)
        {
            yawRateSetpoint = clamp(wrap_180(headingTarget - heading) * HEADING_KP, MAX_YAW_RATE_DPS);
        }

        if (!flying)
        {
            for (Pid *pid : ratePids)
            {
                pid->reset();
            }
            rollOutput = pitchOutput = yawOutput = 0;
            lastPidUs = 0;
            // Position hold's wind correction is kept through the flight, but each flight starts afresh.
            northVelPid.reset();
            eastVelPid.reset();
        }
        else if (freshImu)
        {
            float dt = lastPidUs == 0 ? LOOP_PERIOD_US / 1e6f : (now - lastPidUs) / 1e6f;
            if (dt > MAX_PID_DT_S)
            {
                dt = MAX_PID_DT_S;
            }
            lastPidUs = now;

            rollOutput = rollRatePid.update(rollRateSetpoint, rollRate, dt, mixerSaturated);
            pitchOutput = pitchRatePid.update(pitchRateSetpoint, pitchRate, dt, mixerSaturated);
            yawOutput = yawRatePid.update(yawRateSetpoint, yawRate, dt, mixerSaturated);

            if (altHold)
            {
                // Outside its deadband the throttle stick sets a climb rate; back inside it, the drone holds
                // the altitude it had reached.
                float climbStick = climb_stick(target.throttle, hoverStick);
                float climbRateSetpoint;
                if (climbStick != 0)
                {
                    climbRateSetpoint = climbStick * MAX_CLIMB_RATE_MS;
                    altTarget = altitude.altitude();
                }
                else
                {
                    climbRateSetpoint = clamp((altTarget - altitude.altitude()) * ALT_KP, MAX_CLIMB_RATE_MS);
                }
                climbOutput = climbRatePid.update(climbRateSetpoint, altitude.climbRate(), dt);
            }

            if (posHold)
            {
                // The roll and pitch sticks set a speed relative to the way the drone points. Centred, it
                // brakes, then holds the point where it slowed down.
                float forwardStick = -shape_stick(target.pitch, 0);
                float rightStick = shape_stick(target.roll, 0);
                float cosHeading = cosf(heading / RAD_TO_DEG), sinHeading = sinf(heading / RAD_TO_DEG);
                bool stickFlying = forwardStick != 0 || rightStick != 0;
                float speedNorth, speedEast;
                if (stickFlying)
                {
                    posHeld = false;
                    speedNorth = (forwardStick * cosHeading - rightStick * sinHeading) * MAX_POS_SPEED_MS;
                    speedEast = (forwardStick * sinHeading + rightStick * cosHeading) * MAX_POS_SPEED_MS;
                }
                else
                {
                    if (!posHeld && hypotf(north.velocity(), east.velocity()) < POS_CAPTURE_SPEED_MS)
                    {
                        posHeld = true;
                        posTarget[0] = north.position();
                        posTarget[1] = east.position();
                    }
                    speedNorth = posHeld ? (posTarget[0] - north.position()) * POS_KP : 0;
                    speedEast = posHeld ? (posTarget[1] - east.position()) * POS_KP : 0;
                    float speed = hypotf(speedNorth, speedEast);
                    if (speed > MAX_POS_SPEED_MS)
                    {
                        speedNorth *= MAX_POS_SPEED_MS / speed;
                        speedEast *= MAX_POS_SPEED_MS / speed;
                    }
                }
                // The integrals lean into a steady wind. They only learn it while braking and holding: in
                // flight they would pick up air drag as well, and carry it on past the stop.
                float accelNorth = northVelPid.update(speedNorth, north.velocity(), dt, stickFlying);
                float accelEast = eastVelPid.update(speedEast, east.velocity(), dt, stickFlying);

                // Tilt toward the acceleration wanted: the nose down to speed up forward, the right side down
                // to speed up to the right.
                float accelForward = accelNorth * cosHeading + accelEast * sinHeading;
                float accelRight = -accelNorth * sinHeading + accelEast * cosHeading;
                posPitchSetpoint = clamp(-atanf(accelForward / GRAVITY_MS2) * RAD_TO_DEG, MAX_POS_TILT_DEG);
                posRollSetpoint = clamp(atanf(accelRight / GRAVITY_MS2) * RAD_TO_DEG, MAX_POS_TILT_DEG);
            }
        }
        // After a missed IMU read the PIDs hold their last output.

        float motorUs[MOTOR_COUNT];
        if (armed)
        {
            // In self-level mode, raise the throttle as the drone tilts so it keeps roughly the same lift.
            float throttle = altHold ? fminf(fmaxf(hoverThrottle + climbOutput, 0.0f), 1.0f) : throttle_fraction(target.throttle);
            if (selfLevel && flying)
            {
                throttle = fminf(throttle / fmaxf(tiltCos, 1.0f / TILT_COMP_MAX), 1.0f);
            }
            float throttleUs = MOTOR_IDLE_US + throttle * (MOTOR_MAX_US - MOTOR_IDLE_US);
            mixerSaturated = mix(throttleUs, rollOutput, pitchOutput, yawOutput, MOTOR_IDLE_US, MOTOR_MAX_US, motorUs);
        }
        else
        {
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                motorUs[i] = MOTOR_STOP_US;
            }
            mixerSaturated = false;
        }
        motors_write(motorUs);

        if (now - lastDebugPrintUs >= 200000)
        {
            if (IMU_DEBUG_PRINT && !armed)
            {
                printf("roll %6.1f deg %7.1f deg/s | pitch %6.1f deg %7.1f deg/s | yaw rate %7.1f deg/s | "
                       "tilt %5.1f deg | gyro %d updates/s | altitude %6.2f m %5.2f m/s%s\n",
                       rollAngle, rollRate, pitchAngle, pitchRate, yawRate, tiltAngle, gyroUpdates * 5,
                       altitude.altitude(), altitude.climbRate(), baroOk ? "" : " (no barometer reading)");
                float foundOffsets[3];
                compassCal.offsets(foundOffsets);
                printf("heading %6.1f deg (compass %6.1f deg, field %5.1f uT%s) | magnetometer offsets found so far "
                       "%6.1f %6.1f %6.1f uT | GPS %s, %d satellites, within %.1f m, %.2f %.2f m/s north/east\n",
                       heading, compassHeading, magField, compassRecent ? "" : ", not usable", foundOffsets[0],
                       foundOffsets[1], foundOffsets[2], !gpsReceiving ? "no data" : (gpsOk ? "fix" : "no fix"),
                       gps.satellites, gps.horizontalAccuracy, north.velocity(), east.velocity());
            }
            lastDebugPrintUs = now;
            gyroUpdates = 0;
        }
        loopCount++;

        nextLoop = delayed_by_us(nextLoop, LOOP_PERIOD_US);
        sleep_until(nextLoop);
    }
}
