//
// Created by 31663 on 2026/4/1.
//

#include "Zebra.h"

extern Point centerLine[expRoadCount];
extern int16 centerLineCount, RUN_MODE, LCorner, RCorner;
static uint8 count, add = 1;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];


uint8 haveLine() {
    int16 i = 0;
    while (i < LBorderCount && LBorder[i].x > ZEBRA_TRIGGER_POS) {
        if (findZebra(i)) return 1;
        i += 5;
    }
    return !LBorderCount || LBorder[i].x > ZEBRA_TRIGGER_POS;
}

void runZebra(uint8 have) {
    centerLineDistance = 17;
    alongLine(centerLine, centerLineCount, getDynamicAimPos());
    if (count == 3) {
        stop:
        system_delay_ms(100);
        handle = 1;
        pwm_set_duty(PWMF_CH2_PA3, 0);
        pwm_set_duty(PWMF_CH1_PA1, 0);
        setSpeed = 0;
    } else {
        if (add) {
            count++;
            if (count == 3) goto stop;
            add = 0;
        }
        if (!LCorner && !RCorner && LBorderCount && !have && !haveLine()) {
            RUN_MODE = FORWARD;
            add = 1;
        }
    }
}
