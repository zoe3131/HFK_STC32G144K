//
// Created by 31663 on 2026/3/29.
//

#include "LRound.h"

#define ROUND_READY 1
#define ROUND_START 2
#define ROUND_IN 3
#define ROUND_END 4
#define ROUND_EXIT 5

static uint8 ROUND_MODE = ROUND_READY, canStart = 0, canIn = 0, canFinish = 0;
extern int16 centerLineCount, RUN_MODE, RRoadCount, LRoadCount, ALONG;
extern uint8 mustStop;
extern Point centerLine[expRoadCount], RBorder[expRoadCount];

void runLRound(int16 LCorner, int16 RCorner) {
    if (ROUND_MODE == ROUND_READY) {
        handle = 1;
        setSpeed = 250;
        gpio_set_level(IO_P65, 1);
        if (centerLineCount < 5) {
            smoothStop = 1;
            return;
        } else smoothStop = 0;
        alongLine(centerLine, centerLineCount, getDynamicAimPos());
        if (!LRoadCount)
            canStart = 1;
        else if (LRoadCount > 10 && !LCorner && canStart) {
            ROUND_MODE = ROUND_START;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_START) {
        handle = 0;
        gpio_set_level(IO_P65, 0);
        ALONG = ALONG_L;
        if (centerLineCount > 10)
            alongLine(centerLine, centerLineCount, 28);
        else
            setServoDuty(22);
        if (!RRoadCount)
            canIn = 1;
        else if (RRoadCount > 10 && !RCorner && canIn) {
            centerLineDistance = 26;
            ROUND_MODE = ROUND_IN;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_IN) {
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, getDynamicAimPos());
        if (centerLineCount < 5) smoothStop = 1;
        else smoothStop = 0;
        if (RCorner > 3 && centerLineCount < 17) {
            centerLineDistance = 8;
            ROUND_MODE = ROUND_END;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_END) {
        ALONG = ALONG_L;
        if (centerLineCount > 5)
            alongLine(centerLine, centerLineCount, 17);
        else
            setServoDuty(28);
        if (!RCorner && RRoadCount > 25 && RBorder[RRoadCount - 1].y < 133) {
            centerLineDistance = 17;
            ROUND_MODE = ROUND_EXIT;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_EXIT) {
        ALONG = ALONG_R;
        if (centerLineCount < 5) {
            smoothStop = 1;
        } else smoothStop = 0;
        alongLine(centerLine, centerLineCount, 40);
        if (!LRoadCount)
            canFinish = 1;
        else if (LRoadCount > 10 && !LCorner && canFinish) {
            ROUND_MODE = ROUND_READY;
            RUN_MODE = FORWARD;
            smoothStop = 0;
            canFinish = canStart = canIn = 0;
        }
    }
}