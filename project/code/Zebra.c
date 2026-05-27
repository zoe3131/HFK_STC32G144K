//
// Created by 31663 on 2026/4/1.
//

#include "Zebra.h"

extern Point centerLine[expRoadCount];
extern int16 centerLineCount, RUN_MODE, LCorner, RCorner;
uint8 canEnd, count, add;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];


uint8 haveLine() {
    int16 pos = 0;
    if (LBorderCount > RBorderCount) {
        while (pos < LBorderCount && LBorder[pos].x > ZEBRA_TRIGGER_POS)
            pos += 3;
        pos = func_min(pos, LBorderCount - 1);
        for (; pos > 0; pos -= 4)
            if (isZebra(pos, ALONG_L)) return 1;
        return 0;
    }
    while (pos < RBorderCount && RBorder[pos].x > ZEBRA_TRIGGER_POS)
        pos += 3;
    pos = func_min(pos, RBorderCount - 1);
    for (; pos > 0; pos -= 4)
        if (isZebra(pos, ALONG_R)) return 1;
    return 0;
}

void runZebra(uint8 have) {
    alongLine(centerLine, centerLineCount, getDynamicAimPos());
    if (count == 2) {
        stop:
        smoothStop = 0;
        handle = 1;
        setSpeed = 0;
        ips200_show_string(0, 0, "FINISH");
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
