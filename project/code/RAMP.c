//
// Created by 31663 on 2026/4/22.
//

#include "RAMP.h"

static uint8 canEnd = 0, readyEnd = 0;
extern int16 RUN_MODE, pitch;

void runRamp() {
    //(func_abs(LRoadCount - RRoadCount) < 5 && LRoadCount > 30 && LRoadCount < 40 && RBorder[30].y - LBorder[30].y > 75 && RBorder[30].y > 124 && LBorder[30].y < 64)
    Point LStart = getLStart(), RStart = getRStart(), LBorder[expRoadCount], RBorder[expRoadCount], tarPos[6];
    int16 LRoadCount = expRoadCount, RRoadCount = expRoadCount, i, j;
    gpio_set_level(IO_P65, 1);
    setSpeed = 200;
    getLBorder(LBorder, LStart, &LRoadCount);
    getRBorder(RBorder, RStart, &RRoadCount);
    if (func_min(LRoadCount, RRoadCount) < 30) {
        smoothStop = 1;
        return;
    } else smoothStop = 0;
    for (i = 0; i < LRoadCount; i++)
        if (LBorder[i].x == rampPos) break;
    for (j = 0; j < RRoadCount; j++)
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
