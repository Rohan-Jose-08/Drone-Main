#include <stdio.h>
#include "pico/stdlib.h"
#include <RF24.h>
#include "include/bno.h"
#include "include/PID_v1.h"

#define CE_PIN 20
#define CSN_PIN 17

#define UART_ID uart0
#define BAUD_RATE 115200
#define TX_PIN 21
#define RX_PIN 22

RF24 radio(CE_PIN, CSN_PIN);

struct JoystickData
{
    int throttle;
    int yaw;
    int pitch;
    int roll;
};

float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
float rollInput, pitchInput, yawInput;
float rollOutput, pitchOutput, yawOutput;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 1.5, 0.05, 0.01, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1.5, 0.05, 0.01, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 2.0, 0.1, 0.01, DIRECT);


int main()
{
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    bno055_t bno;

    bno.dev_addr = 0x28;
    bno.bus_write = bno055_bus_write;
    bno.bus_read = bno055_bus_read;
    bno.delay_msec = bno055_delay_msek;

    s8 com_rslt = bno055_init(&bno);
    if (com_rslt != 0)
    {
        printf("BNO055 initialization failed with error code: %d\n", com_rslt);
        return 1;
    }
    com_rslt = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if (com_rslt != 0)
    {
        printf("Failed to set operation mode, error: %d\n", com_rslt);
    }
    sleep_ms(1000);

    struct bno055_euler_float_t eulerData;
    bno055_convert_float_euler_hpr_deg(&eulerData);

    unsigned char accel_calib_status = 0;
    unsigned char gyro_calib_status = 0;
    unsigned char mag_calib_status = 0;
    unsigned char sys_calib_status = 0;
    JoystickData data;
    JoystickData outputData;

    bno055_get_accel_calib_stat(&accel_calib_status);
    bno055_get_mag_calib_stat(&mag_calib_status);
    bno055_get_gyro_calib_stat(&gyro_calib_status);
    bno055_get_sys_calib_stat(&sys_calib_status);

    if (!radio.begin())
    {
        printf("NRF24L01 failed to initialize!\n");
        return 1;
    }

    radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
    radio.setPALevel(RF24_PA_HIGH);
    radio.startListening();



    while (true)
    {

        if (radio.available())
        {
            radio.read(&data, sizeof(data));
    
            
        }

        rollSetpoint = data.roll;
        pitchSetpoint = data.pitch;
        yawSetpoint = data.yaw;



        uint8_t datas[6] = {0};
        com_rslt = bno055_bus_read(bno.dev_addr, 0x1A, datas, 6);

        if (com_rslt == 0)
        {

            int16_t yaw_raw = (int16_t)((datas[1] << 8) | datas[0]);
            int16_t roll_raw = (int16_t)((datas[3] << 8) | datas[2]);
            int16_t pitch_raw = (int16_t)((datas[5] << 8) | datas[4]);

             yawInput = yaw_raw / 16.0f;
             rollInput = roll_raw / 16.0f;
             pitchInput = pitch_raw / 16.0f;
        }

        rollPID.Compute();
        pitchPID.Compute();
        yawPID.Compute();

        outputData.roll = rollOutput;
        outputData.pitch = pitchOutput;
        outputData.yaw = yawOutput;


        uart_puts(UART_ID, (char *)&outputData);
        
        sleep_ms(10);
     
    }
}
