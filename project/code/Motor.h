//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#define servoFreq 200
#define FORWARD 114
#define CROSS 514
#define R_ROUND 1919
#define L_ROUND 810
#define ZEBRA 520
#define RAMP 1314
#define BLOCK 521
#define ALONG_L 88
#define ALONG_R 99

extern int16 setSpeed, handle;
void motorInit(int16 p, int16 i, int16 d,int16 am, int16 sp, int16 dh);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getDynamicAimPos();
extern int8 smoothStop;

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
