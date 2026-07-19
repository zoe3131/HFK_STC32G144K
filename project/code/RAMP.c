//
// Created by 31663 on 2026/4/22.
//

#include "RAMP.h"

extern int16 RUN_MODE;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];
uint8 canEnd, canNotInRamp;

void runRamp() {
    Point tarPos[6];
    int16 i, j, k;
    gpio_set_level(IO_P90, 1);
    if (pitch <= -5) {
        canEnd = 1;
        setSpeed = 350;
    } else if (pitch > -2 && pitch < 2 && canEnd && dl1b_distance_mm > 500) {
        RUN_MODE = FORWARD;
        wireless_uart_send_string("exit\r\n");
        centerLineDistance = 18;
        canEnd = handle = 0;
        canNotInRamp = 20;
        enableDifSpeed = 1;
        gpio_set_level(IO_P90, 0);
    } else if (!canEnd) {
        handle = 1;
        //wireless_uart_send_string("ramp\n");
        setSpeed = 200;
        enableDifSpeed = 0;
    }
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
}