//
// Created by 31663 on 2026/4/10.
//

#include "Block.h"
static uint8 added = 0;
extern uint8 ALONG;
static int16 fixedLCorner, fixedRCorner;
extern int16 RUN_MODE, centerLineCount;
extern Point centerLine[expRoadCount];

uint8 canEnd() {
    int16 i;
    if (fixedLCorner) {
        for (i = 0; i < RBorderCount; i += 10)
            if (mt9v03x_image[RBorder[i].x][RBorder[i].y - borderOffset[RBorder[i].x]] < blockThreshold) return 0;
    } else {
        for (i = 0; i < LBorderCount; i += 10)
            if (mt9v03x_image[LBorder[i].x][LBorder[i].y + borderOffset[LBorder[i].x]] < blockThreshold) return 0;
    }
    return 1;
}

void runBlock(int16 LCorner, int16 RCorner) {
    gpio_set_level(IO_P65, 1);
    if (!added) {
        fixedLCorner = LCorner;
        fixedRCorner = RCorner;
        added = 1;
    }
    ALONG = fixedLCorner ? ALONG_R : ALONG_L;
    if (centerLineCount < 10)
        smoothStop = 1;
    else {
        smoothStop = 0;
        alongLine(centerLine, centerLineCount, 20);
    }
    if (canEnd()) {
        added = smoothStop = 0;
        gpio_set_level(IO_P65, 0);
        RUN_MODE = FORWARD;
        centerLineDistance = 18;
    }
}