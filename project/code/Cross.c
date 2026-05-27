//
// Created by 31663 on 2026/3/29.
//

#include "Cross.h"

#define CROSS_READY 1
#define CROSS_IN 2
extern uint8 outputImage[MT9V03X_H][MT9V03X_W], crossDis;
static uint8 halfBlock = block / 2, QW = 45, DQW = 143, m;
static int8 CROSS_MODE = CROSS_READY, canEnd = 0;
static int16 alongR, crossThreshold;
extern int16 centerLineCount, RUN_MODE, LRoadCount, RRoadCount, LCorner, RCorner;
extern Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
static Point start;

Point getCrossRStart() {
    int16 data i = 90, data j = DQW, k;
    Point pos, startT = getRStart();
    if (startT.y || startT.x) {
        i = startT.x - 2;
        j = startT.y + 1;
        if (RBorderCount > 15) {
            m = MT9V03X_W - 1;
            for (k = 0; k < RBorderCount; k++)
                if (m >= RBorder[k].y) m = RBorder[k].y;
                else {
                    i = RBorder[k].x - 2;
                    j = RBorder[k].y;
                    break;
                }
            DQW = m;
        }
    }
    while (i >= 0 && mt9v03x_image[i][j] <= crossThreshold) i -= 3;
    j = DQW;
    while (i >= 0 && mt9v03x_image[i][j] > crossThreshold) i--;
    if (i < halfBlock) {
        pos.x = 0;
        pos.y = 0;
    } else {
        pos.x = i + 1;
        pos.y = j;
    }
    return pos;
}

Point getCrossLStart() {
    int16 data i = 90, data j = QW, k;
    Point pos, startT = getLStart();
    if (startT.y || startT.x) {
        i = startT.x - 2;
        j = startT.y - 1;
        if (LBorderCount > 15) {
            m = 0;
            for (k = 0; k < LBorderCount; k++)
                if (m <= LBorder[k].y) m = LBorder[k].y;
                else {
                    i = LBorder[k].x - 2;
                    j = LBorder[k].y;
                    break;
                }
            QW = m;
        }
    }
    while (i >= 0 && mt9v03x_image[i][j] <= crossThreshold) i -= 3;
    j = QW;
    while (i >= 0 && mt9v03x_image[i][j] > crossThreshold) i--;
    if (i < halfBlock) {
        pos.x = 0;
        pos.y = 0;
    } else {
        pos.x = i + 1;
        pos.y = j;
    }
    return pos;
}

void runCross() {
    smoothStop = 0;
    if (CROSS_MODE == CROSS_READY) {
        gpio_set_level(IO_P65, 1);
        if (centerLineCount < 10) {
            crossThreshold = threshold;
            alongR = LBorderCount < RBorderCount;
            CROSS_MODE = CROSS_IN;
        }
        alongLine(centerLine, centerLineCount, getDynamicAimPos());
    } else if (CROSS_MODE == CROSS_IN) {
        Point border[expRoadCount], farCenterLine[expRoadCount];
        int16 roadCount = expRoadCount, farCenterLineCount, corner, i, j;
        if (alongR) {
            start = getCrossRStart();
            getRBorder(border, start, &roadCount);
        } else {
            start = getCrossLStart();
            getLBorder(border, start, &roadCount);
        }
        getGodRoadLine(border, roadCount);
        resampleRoad(border, &roadCount, 0, 3);
        corner = findCorner(border, roadCount);
        filterRoad(border, roadCount);
        farCenterLineCount = roadCount;
        if (alongR)
            getCenterLineByRBorder(border, farCenterLine, farCenterLineCount);
        else getCenterLineByLBorder(border, farCenterLine, farCenterLineCount);
        if (!LRoadCount && !RRoadCount) canEnd = 1;
        if (corner && farCenterLine[corner].x > 50) corner = func_min(corner + 3, farCenterLineCount);
        else corner = 0;
        farCenterLine[corner].x = MT9V03X_H - 1;
        farCenterLine[corner].y = MT9V03X_W / 2 - 3;
        resampleRoad(farCenterLine, &farCenterLineCount, corner, 2);
        filterRoad(farCenterLine, farCenterLineCount);
        alongLine(farCenterLine, farCenterLineCount, 25);
        if (centerLineCount > 20 && canEnd) {
            canEnd = 0;
            CROSS_MODE = CROSS_READY;
            RUN_MODE = FORWARD;
            gpio_set_level(IO_P65, 0);
            return;
        }
        if (crossDis) {
            ips200_displayimage03x(mt9v03x_image, MT9V03X_W, MT9V03X_H, 0, 0);
            ips200_draw_point(start.y, start.x, RGB565_RED);
            ips200_show_int16(0, MT9V03X_H + 1, corner);
            ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, farCenterLineCount);
            for (i = 0; i < MT9V03X_H; i++)
                for (j = 0; j < MT9V03X_W; j++)
                    outputImage[i][j] = 0;
            for (i = 0; i < farCenterLineCount; i++)
                outputImage[farCenterLine[i].x][farCenterLine[i].y] = 255;
            for (i = 0; i < roadCount; i++)
                outputImage[border[i].x][border[i].y] = 255;
            ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
        }
    }
}
