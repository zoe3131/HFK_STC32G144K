//
// Created by 31663 on 2026/4/22.
//

#include "RAMP.h"

static uint8 canEnd = 0, readyEnd = 0;
extern int16 RUN_MODE, pitch;

void runRamp() {
    Point tarPos[6];
    int16 i, j;
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
    for (i = 0; i <= 5; i++) {
        tarPos[i].x = rampPos;
        tarPos[i].y = LBorder[i].y + RBorder[j].y >> 1;
    }
    getGodRoadLine(tarPos, 6);
    alongLine(tarPos, 6, 5);
    handle = 1;
    setSpeed = 200;
    if (dl1b_distance_mm < 400) {
        if (readyEnd)
            canEnd = 1;
    } else if (dl1b_distance_mm > 600) {
        readyEnd = 1;
        if (canEnd && pitch > -10 && pitch < 10) {
            gpio_set_level(IO_P65, 0);
            RUN_MODE = FORWARD;
            canEnd = handle = readyEnd = 0;
        }
    }
}
