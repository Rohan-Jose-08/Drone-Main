# BAE Drone Main

Quadcopter flight controller firmware for the Raspberry Pi Pico 2 (RP2350). It runs a 400 Hz control loop and reads a BNO055 IMU. Commands arrive over an NRF24L01 radio, and the four ESCs are driven directly with hardware PWM. It has a self-level and an acro mode. With an optional BMP388/BMP390 barometer it adds altitude hold, and with a u-blox GPS, position hold.

## Features

- **Stabilisation:** cascaded control, an angle loop over a rate PID loop, at 400 Hz.
  - The rate PIDs take the derivative on the measurement, filter it, and stop the integral winding up.
  - The quad-X mixer keeps the roll/pitch difference when a motor reaches its limit.
- **Estimation:**
  - The gyro bias is calibrated at power-up.
  - The gyro has a low-pass filter and an optional notch filter.
  - Tilt comes from its own complementary (Mahony-style) filter. With GPS, that filter takes the drone's own acceleration out of the accelerometer reading.
  - Heading comes from the gyro, corrected by the compass.
- **Modes:**
  - Self-level, or acro for tuning the rate loop on a test rig.
  - Heading hold.
  - Altitude hold, with a barometer.
  - Position hold, with GPS and the compass.
- **Safety:**
  - Arming and disarming by stick commands, with checks before arming.
  - Staged radio failsafe.
  - Radio packets carry a sequence number and a checksum.
  - Disarms on a crash tilt or if the IMU stops updating.
  - Hardware watchdog.
  - The ESCs get a stop pulse from the moment of power-up.

## Hardware and wiring

| Part | Connection | Notes |
|---|---|---|
| Raspberry Pi Pico 2 | | USB serial carries the status and debug output |
| BNO055 IMU | I2C0: SDA GP4, SCL GP5, address 0x28 | Raw accelerometer, gyro and magnetometer (AMG mode). Mounting is set in the code, see [Configuration](#configuration) |
| NRF24L01 radio | SPI0: SCK GP18, MOSI GP19, MISO GP16, CSN GP17, CE GP20 | |
| 4 ESCs | GP6 to GP9, see [Motor layout](#motor-layout) | Standard PWM, 1000 to 2000 µs at 400 Hz |
| BMP388 or BMP390 barometer (optional) | Same I2C bus as the BNO055, address 0x77 or 0x76 | Needed for altitude and position hold |
| u-blox GPS, M8, M9 or M10 (optional) | UART0: GP0 to the GPS's RX, GP1 to its TX | Needed for position hold. Configured automatically at every power-up |

The barometer and GPS are detected at start-up. Without them the drone flies as before, and the modes that need them can't be turned on.

## Motor layout

Quad X, seen from above with the front at the top. Diagonal motors spin the same way.

```
   M4  GP9  front left,  clockwise        M2  GP7  front right, anticlockwise
   M3  GP8  rear left,   anticlockwise    M1  GP6  rear right,  clockwise
```

Sign conventions: positive roll lowers the right side, positive pitch raises the nose, and positive yaw turns the drone clockwise seen from above.

## Transmitter protocol

The transmitter code isn't in this repository. It has to send this packet as a fixed-size NRF24 payload of 11 bytes, little-endian, to pipe address `0xF0F0F0F0E1`:

```cpp
struct __attribute__((packed)) ControlPacket
{
    uint8_t sequence;       // add 1 on every packet; a repeated value isn't counted as a live link
    int16_t throttle;       // 0..1023
    int16_t yaw;            // 0..1023, centred at 512; higher is right
    int16_t pitch;          // 0..1023, centred at 512
    int16_t roll;           // 0..1023, centred at 512; higher is right
    uint8_t switches;       // bit 0: altitude hold, bit 1: position hold
    uint8_t checksum;       // XOR of the 10 bytes before it
};
```

Send at least every 50 ms; 50 to 100 Hz is better. A 3-position switch maps naturally onto `switches`: off is `0x00`, altitude hold `0x01`, position hold `0x02`. If your sticks use a different range, change `STICK_CENTER`, `STICK_HALF_RANGE` and the `THROTTLE_` settings.

## Building and flashing

You need the Raspberry Pi Pico SDK 2.1.1 and the Arm GNU toolchain 14.2. The [Raspberry Pi Pico extension for VS Code](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) installs both, and the project is set up for it.

1. **Fetch the RF24 library.** `lib/RF24` is recorded as a submodule, but the repository has no `.gitmodules` file, so a plain clone leaves the folder empty:

   ```sh
   rmdir lib/RF24
   git clone https://github.com/nRF24/RF24.git lib/RF24
   git -C lib/RF24 checkout 929a5aa
   ```

2. **Build.** Either use the extension's **Compile Project** command, or from the command line:

   ```sh
   cmake -S . -B build -G Ninja
   cmake --build build
   ```

   Outside VS Code, set `PICO_SDK_PATH`, and put the toolchain and Ninja on your `PATH`. Under WSL, picotool needs `libusb-1.0-0` installed (`sudo apt install libusb-1.0-0`).

3. **Flash.** Hold BOOTSEL while plugging in the Pico 2, then copy `build/BAE-Drone-Main.uf2` onto the drive that appears.

## First-time setup (propellers off)

1. **Watch the serial output.** Open the USB serial port. With `IMU_DEBUG_PRINT` set to 1, the debug lines are printed 5 times a second while disarmed.
2. **Keep it still at power-up** until "Gyro calibrated" appears, about a second after "Ready, disarmed". Moving it starts the calibration again.
3. **Check the mounting.**
   - Sitting level, roll and pitch should read near 0.
   - Lowering the right side makes roll and roll rate positive.
   - Raising the nose makes pitch and pitch rate positive.
   - Turning clockwise seen from above makes the yaw rate positive.
   - If any of these is wrong, fix `FORWARD_AXIS`, `RIGHT_AXIS`, `DOWN_AXIS` and their signs.
   - The gyro should report about 400 updates/s.
4. **Check the motors.**
   - Arm, raise the throttle a little, and confirm each motor's position and spin direction against the [layout](#motor-layout).
   - Tilting the frame by hand should speed up the motors on the low side.
   - If a motor stutters at idle, raise `MOTOR_IDLE_US`.
5. **Calibrate the compass** (needed for position hold).
   - With the motors off, turn the drone slowly through every orientation: all the way round level, nose up and down, on each side, and upside down.
   - Copy the "magnetometer offsets found so far" into `MAG_OFFSET_X/Y/Z`, and set `MAG_DECLINATION_DEG` for where you fly.
   - After reflashing, the heading should read near 0 facing north and 90 facing east.
   - It should stay steady from idle to full throttle. If it moves more than a few degrees, the compass is too close to the power wiring.
6. **Barometer:** the altitude should read about 0 at power-up. Lifting the drone 1 m should raise it by about 1.
7. **GPS, outdoors:** "GPS connected", then "GPS fix: N satellites", should appear. Standing still, the north/east velocities should sit near 0.
8. **Failsafe:** arm, raise the throttle above `FAILSAFE_THROTTLE`, then switch the transmitter off. The motors should drop back after 0.25 s and stop after 2 s.

## Flying

**Arming:** throttle low (at most 50), then hold yaw fully right for 1 s. Arming is refused, with the reason printed, unless:
- the IMU is responding,
- the gyro has been calibrated,
- the accelerometer reads about 1 g,
- the drone is within 25° of level.

**Disarming:** throttle low, then hold yaw fully left for 1 s.

**Modes**

- **Self-level (the default):** roll and pitch sticks set an angle of up to 30°. Yaw sets a rotation rate of up to 150°/s.
- **Acro (`ACRO_MODE 1`):** roll and pitch sticks set rotation rates instead, of up to 200°/s. For tuning the rate loop on a test rig.
- **Heading hold** (self-level only): once the yaw stick is centred and the drone has stopped turning, it holds that heading.
- **Altitude hold** (switch bit 0):
  - Turn it on while hovering. The throttle stick's position becomes the hover point, and moving it up or down climbs or descends at up to 1 m/s.
- **Position hold** (switch bit 1, which includes altitude hold):
  - Turn it on while hovering. Roll and pitch then fly the drone at up to about 3 m/s relative to the way it points; the pitch stick direction that tips the nose down in self-level flies it forward.
  - Centre the sticks and it brakes, then holds where it stopped.
  - It needs a 3D GPS fix with at least 6 satellites and a usable compass reading.
  - If it loses either, it drops back to altitude hold.

**Switching the holds off:** turning a switch off hands control back to the sticks. A hold also stops by itself in these cases:
- the throttle stick is pulled to idle,
- the radio link is lost,
- the sensor it depends on fails.

After any automatic stop it stays off until its switch is turned off and on again.

**Radio failsafe**
- After 0.25 s without a valid packet, the drone levels out and descends, with the throttle capped at `FAILSAFE_THROTTLE`. Set that just below your hover throttle.
- After 2 s it disarms.

**Automatic disarm:**
- tilt past 60°,
- the IMU not updating for 50 ms,
- 2 s without the radio link.

## Configuration

All settings are at the top of [BAE-Drone-Main.cpp](BAE-Drone-Main.cpp), mostly as `#define`s, each with a comment. The ones to look at first:

| Setting | What it is |
|---|---|
| `FORWARD_AXIS`, `RIGHT_AXIS`, `DOWN_AXIS` and signs | How the BNO055 is mounted |
| `ROLL_TRIM_DEG`, `PITCH_TRIM_DEG` | Level trim |
| `MOTOR_PINS`, `ESC_RATE_HZ`, `MOTOR_IDLE_US` | ESC outputs (use 50 Hz for ESCs that need it) |
| `STICK_CENTER`, `STICK_HALF_RANGE`, `THROTTLE_*`, `FAILSAFE_THROTTLE` | Transmitter scale and failsafe throttle |
| `ACRO_MODE`, `MAX_ANGLE_DEG`, `MAX_*_RATE_DPS`, `STICK_EXPO` | Flight mode and stick feel |
| `RATE_*_KP/KI/KD`, `ANGLE_KP` | Attitude gains |
| `GYRO_LPF_HZ`, `GYRO_NOTCH_HZ` | Gyro filters (set the notch once logs show the motor vibration frequency) |
| `MAG_OFFSET_*`, `MAG_DECLINATION_DEG` | Compass calibration |
| `CLIMB_*`, `ALT_KP`, `MAX_CLIMB_RATE_MS` | Altitude hold |
| `POS_KP`, `VEL_KP`, `VEL_KI`, `MAX_POS_SPEED_MS`, `MAX_POS_TILT_DEG` | Position hold |
| `IMU_DEBUG_PRINT` | Debug output while disarmed |

## How it works

```
sticks → modes → [position → velocity] → angle target → [angle P] → rate target → [rate PID] → mixer → ESCs
                 [altitude → climb rate] → throttle ───────────────────────────────────────────↗
```

Each loop runs inside a faster one. The inner rate loop, driven by the gyro, does most of the stabilising. The angle loop turns an angle error into a rate target, which gives self-levelling. Altitude and position hold sit on top.

The estimates behind them are all complementary filters, each one fast sensor kept from drifting by a slow one:

- **Tilt:** gyro, corrected by the accelerometer.
- **Heading:** gyro, corrected by the compass.
- **Altitude and climb rate:** vertical acceleration, corrected by the barometer. It also learns the accelerometer's offset.
- **Position and velocity:** acceleration, corrected by GPS. Fixes are compared with the estimate from when they were measured, to allow for GPS lag.

## Source layout

| File | Contents |
|---|---|
| [BAE-Drone-Main.cpp](BAE-Drone-Main.cpp) | Settings, start-up, and the control loop: sensors, safety, modes and output |
| [pid.cpp](pid.cpp) | PID controller |
| [mixer.cpp](mixer.cpp) | Quad-X mixer with desaturation |
| [motors.cpp](motors.cpp) | ESC PWM output |
| [filters.cpp](filters.cpp) | Biquad low-pass and notch filters |
| [attitude.cpp](attitude.cpp) | Tilt estimate |
| [altitude.cpp](altitude.cpp) | Altitude and climb rate estimate |
| [position.cpp](position.cpp) | Horizontal position and velocity estimate, frame rotations |
| [compass.cpp](compass.cpp) | Tilt-compensated compass heading and calibration |
| [baro.cpp](baro.cpp) | BMP388/BMP390 driver |
| [gps.cpp](gps.cpp) | u-blox UBX driver: configuration and NAV-PVT parsing |
| [bno.cpp](bno.cpp), [bno055.c](bno055.c) | BNO055 I2C glue and Bosch's BNO055 driver |

## Third-party code

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), BSD-3-Clause.
- Bosch Sensortec BNO055 driver ([bno055.c](bno055.c), [include/bno055.h](include/bno055.h)), BSD-3-Clause.
- [RF24](https://github.com/nRF24/RF24), GPL-2.0, fetched into `lib/RF24`.
- The barometer compensation follows the BMP388/BMP390 datasheet and Bosch's [BMP3 Sensor API](https://github.com/boschsensortec/BMP3_SensorAPI). The GPS messages follow u-blox's interface descriptions.
