//
// Created by 31663 on 2026/4/1.
//

#include "Zebra.h"

extern Point centerLine[expRoadCount];
extern int16 centerLineCount, RUN_MODE, LCorner, RCorner;
uint8 canEnd, count, add;

uint8 haveLine() {
    int16 pos;
    if (LBorderCount < RBorderCount) {
        pos = LBorderCount - 1;
        while (LBorder[pos].x < ZEBRA_TRIGGER_POS)
            pos -= 3;
        for (pos -= 10; pos >= 5; pos -= 10)
            if (isZebra(pos, ALONG_L)) return 1;
    } else {
        pos = RBorderCount - 1;
        while (RBorder[pos].x < ZEBRA_TRIGGER_POS)
            pos -= 3;
        for (pos -= 10; pos >= 5; pos -= 10)
            if (isZebra(pos, ALONG_R)) return 1;
    }
    return 0;
}

void runZebra(uint8 have) {
    alongLine(centerLine, centerLineCount, getDynamicAimPos());
    if (count == 3) {
        stop:
        smoothStop = 0;
        handle = 1;
        setSpeed = 0;
    } else {
        if (add) {
            count++;
            if (count == 3) goto stop;
            add = 0;
        }
        if (!LCorner && !RCorner && !haveLine() && !have) {
            RUN_MODE = FORWARD;
            add = 1;
            canEnd = 0;
        }
    }
}