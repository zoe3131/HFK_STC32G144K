//
// Created by 31663 on 2026/4/22.
//

#include "RAMP.h"

static uint8 canEnd = 0, readyEnd = 0;
extern int16 RUN_MODE, pitch;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];

void runRamp() {
    Point tarPos[6];
    int16 i, j, k;
    gpio_set_level(IO_P65, 1);
    if (func_min(LBorderCount, RBorderCount) < 30) {
        smoothStop = 1;
        return;
    }
    smoothStop = 0;
    for (i = 0; i < LBorderCount; i++)
        if (LBorder[i].x == rampPos) break;
    for (j = 0; j < RBorderCount; j++)
        if (RBorder[j].x == rampPos) break;
    if (LBorder[i].x != rampPos || RBorder[j].x != rampPos) return;
    for (k = 0; k <= 5; k++) {
        tarPos[k].x = rampPos;
        tarPos[k].y = LBorder[i].y + RBorder[j].y >> 1;
    }
    getGodRoadLine(tarPos, 6);
    alongLine(tarPos, 6, 5);
    handle = 1;
    setSpeed = 170;
    // ips200_show_int16(0, 0, readyEnd);
    // ips200_show_int16(0, 20, canEnd);
    // ips200_show_int16(0, 40, dl1b_distance_mm);
    if (dl1b_distance_mm < 500) {
        if (readyEnd)
            canEnd = 1;
    } else if (dl1b_distance_mm > 650) {
        if (dl1b_distance_mm > 1000) readyEnd = 1;
        if (canEnd) {
            wireless_uart_send_string("exit\r\n");
            gpio_set_level(IO_P65, 0);
            canEnd = handle = readyEnd = 0;
            system_delay_ms(200);
            RUN_MODE = FORWARD;
        }
    }
}
