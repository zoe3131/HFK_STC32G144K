//
// Created by 31663 on 2026/5/30.
//

#include "Imu.h"


int32 alpha = 997, angle;
int16 pitch;
uint8 send_buff[32];

extern int16 c4;

void pitHandlerImu(void) {
    imu660rb_get_acc();
    imu660rb_get_gyro();
    angle = (alpha * (angle + imu660rb_gyro_x * 6993 / 20000) + (1000 - alpha) * (atan2_int(-imu660rb_acc_y, func_abs(imu660rb_acc_z)) * 1000 - 4000)) / 1000;
    pitch = angle / 1000;
    // sprintf(send_buff, "angle: %d\n", pitch);
    // wireless_uart_send_buffer(send_buff, 32);
}

void imuInit() {
    while (imu660rb_init());
    system_delay_ms(100);
    pit_ms_init(TIM0_PIT, 5, pitHandlerImu);
}
