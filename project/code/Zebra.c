//
// Created by 31663 on 2026/4/1.
//

#include "Zebra.h"

extern Point centerLine[expRoadCount];
extern int16 centerLineCount, RUN_MODE, LCorner, RCorner;
uint8 canEnd, count, add;

uint8 haveLine() {
    int16 data i, data ct, data j, data ad;
    for (i = MT9V03X_H - 1; i >= 0; i -= 5) {
        ad = ct = 0;
        for (j = 0; j <= MT9V03X_W - 1; j += 3) {
            if (i > 85 && j > 45 && j < 135) continue;
            if (mt9v03x_image[i][j] < threshold) {
                if (ad) {
                    ct++;
                    if (ct > 4) return 1;
                    ad = 0;
                }
            } else ad = 1;
        }
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