//
// Created by 31663 on 2026/4/10.
//

#include "Block.h"
static uint8 added = 0, reflect[120];
static int16 fixedLCorner, fixedRCorner;
extern int16 RUN_MODE, ALONG, centerLineCount, LRoadCount, RRoadCount;
extern Point centerLine[expRoadCount], LBorder[expRoadCount], RBorder[expRoadCount];

uint8 canEnd() {
    Point start, border[expRoadCount];
    int16 roadCount = expRoadCount, i;
    if (fixedLCorner) {
        start = getRStart();
        getRBorder(border, start, &roadCount);
        for (i = 0; i < roadCount; i += 10)
            if (mt9v03x_image[border[i].x][border[i].y - reflect[border[i].x]] < blockThreshold) return 0;
    } else {
        start = getLStart();
        getLBorder(border, start, &roadCount);
        for (i = 0; i < roadCount; i += 10)
            if (mt9v03x_image[border[i].x][border[i].y + reflect[border[i].x]] < blockThreshold) return 0;
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
        added = 0;
        gpio_set_level(IO_P65, 0);
        RUN_MODE = FORWARD;
        centerLineDistance = 18;
    }
}

static uint8 reflect[120] = {
    24, 25, 26, 27, 28, 29, 30, 31, 31, 32,
    33, 34, 35, 36, 37, 38, 39, 40, 41, 41,
    42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
    52, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 72, 73, 74, 75, 76, 77, 78,
    79, 80, 81, 82, 82, 83, 84, 85, 86, 87,
    88, 89, 90, 91, 92, 92, 93, 94, 95, 96,
    97, 98, 99, 100, 101, 102, 102, 103, 104, 105,
    106, 107, 108, 109, 110, 111, 112, 112, 113, 114,
    115, 116, 117, 118, 119, 120, 121, 122, 122, 123,
    124, 125, 126, 127, 128, 129, 130, 131, 132, 132
};
