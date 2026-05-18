//
// Created by 31663 on 2026/3/29.
//

#include "Cross.h"

#define CROSS_READY 1
#define CROSS_IN 2
extern uint8 outputImage[MT9V03X_H][MT9V03X_W], display, mustStop;
static uint8 halfBlock = block / 2, QW = 45, DQW = 143, m;
static int8 CROSS_MODE = CROSS_READY, canEnd = 0, added = 0;
static int16 alongR, crossThreshold, count;
extern int16 centerLineCount, RUN_MODE, LRoadCount, RRoadCount;
extern Point centerLine[expRoadCount];
static Point line[expRoadCount], start;

Point getCrossRStart() {
    int16 data i = 90, data j = DQW, k;
    Point pos, start = getRStart();
    if (start.y || start.x) {
        i = start.x - 2;
        j = start.y + 1;
        count = expRoadCount;
        getRBorder(line, start, &count);
        if (count > 10) {
            m = MT9V03X_W - 1;
            for (k = 0; k < count; k++)
                m = func_min(m, line[k].y);
            DQW = m;
        }
    }
    while (i >= 0 && mt9v03x_image[i][j] <= crossThreshold) i -= 3;
    j = DQW;
    while (i >= 0 && mt9v03x_image[i][j] > crossThreshold) i--;
    if (i <= halfBlock) {
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
    Point pos, start = getLStart();
    if (start.y || start.x) {
        i = start.x - 2;
        j = start.y - 1;
        count = expRoadCount;
        getLBorder(line, start, &count);
        if (count > 10) {
            m = 0;
            for (k = 0; k < count; k++)
                m = func_max(m, line[k].y);
            QW = m;
        }
    }
    while (i >= 0 && mt9v03x_image[i][j] <= crossThreshold) i -= 3;
    j = QW;
    while (i >= 0 && mt9v03x_image[i][j] > crossThreshold) i--;
    if (i <= halfBlock) {
        pos.x = 0;
        pos.y = 0;
    } else {
        pos.x = i + 1;
        pos.y = j;
    }
    return pos;
}

void runCross() {
    if (CROSS_MODE == CROSS_READY) {
        gpio_set_level(IO_P65, 1);
        if (!added) {
            added = 1;
            alongR = LRoadCount < RRoadCount;
        }
        if (centerLineCount < 10) {
            crossThreshold = threshold;
            CROSS_MODE = CROSS_IN;
        }
        alongLine(centerLine, centerLineCount, getDynamicAimPos());
    } else if (CROSS_MODE == CROSS_IN) {
        Point border[expRoadCount], farCenterLine[expRoadCount];
        int16 roadCount = expRoadCount, farCenterLineCount, corner, i, j;
        gpio_set_level(IO_P65, 0);
        if (centerLineCount > 25 && canEnd) {
            canEnd = added = 0;
            handle = 0;
            CROSS_MODE = CROSS_READY;
            RUN_MODE = FORWARD;
            return;
        }
        if (alongR) {
            start = getCrossRStart();
            getRBorder(border, start, &roadCount);
        } else {
            start = getCrossLStart();
            getLBorder(border, start, &roadCount);
        }
        getGodRoadLine(border, roadCount);
        resampleRoad(border, &roadCount, 0, 3);
        if (roadCount > 100) return;
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
        alongLine(farCenterLine, farCenterLineCount, getDynamicAimPos());

        // ips200_displayimage03x(mt9v03x_image, MT9V03X_W, MT9V03X_H, 0, 0);
        // ips200_draw_point(start.y, start.x, RGB565_RED);
        // ips200_show_int16(0, MT9V03X_H + 1, corner);
        // ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, farCenterLineCount);
        // for (i = 0; i < MT9V03X_H; i++)
        //     for (j = 0; j < MT9V03X_W; j++)
        //         outputImage[i][j] = 0;
        // for (i = 0; i < farCenterLineCount; i++)
        //     outputImage[farCenterLine[i].x][farCenterLine[i].y] = 255;
        // for (i = 0; i < roadCount; i++)
        //     outputImage[border[i].x][border[i].y] = 255;
        // ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
    }
}