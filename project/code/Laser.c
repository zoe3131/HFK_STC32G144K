//
// Created by 31663 on 2026/7/3.
//

#include "Laser.h"

gpio_pin_enum laser[] = {IO_P17, IO_P67, IO_PB2, IO_P44, IO_PB5, IO_PB3, IO_PB4, IO_P91, IO_P16, IO_P14, IO_P65, IO_P45};
uint8 time, i, canStart = 51;
extern int16 currentDuty;
extern uint8 canNotInRamp;

void dismissLaser() {
    for (i = 0; i < 12; i++)
        gpio_set_level(laser[i], 0);
    gpio_set_level(IO_P90, 0);
}

void timer() {
    if (time) {
        if (time == 1)
            dismissLaser();
        time--;
    } else if (canStart < 50) {
        canStart++;
    }
    if (canNotInRamp) canNotInRamp--;
}

void laserInit() {
    pit_ms_init(TIM5_PIT, 1, timer);
    for (i = 0; i < 12; i++)
        gpio_init(laser[i], GPO, 1, GPO_PUSH_PULL);
}

void startLaser(uint8 i) {
    if (canStart < 50 || i <= 75 || i >= 111) return;
    gpio_set_level(IO_P90, 1);
    canStart = 0;
    time = 10;
    // if (currentDuty < 10 && currentDuty > -10) {
        if (i <= 81) gpio_set_level(laser[0], 1);
        else if (i <= 84) gpio_set_level(laser[1], 1);
        else if (i <= 88) gpio_set_level(laser[2], 1);//1
        else if (i <= 91) gpio_set_level(laser[3], 1);//2
        else if (i <= 95) gpio_set_level(laser[4], 1);//3
        else if (i <= 98) gpio_set_level(laser[5], 1);//4
        else if (i <= 102) gpio_set_level(laser[6], 1);//5
        else if (i <= 105) gpio_set_level(laser[7], 1);
        else gpio_set_level(laser[8], 1);
    // } else if (currentDuty >= 10 && currentDuty < 20) {
    //     if (i <= 80) gpio_set_level(laser[0], 1);
    //     else if (i <= 83) gpio_set_level(laser[1], 1);
    //     else if (i <= 86) gpio_set_level(laser[2], 1);//1
    //     else if (i <= 90) gpio_set_level(laser[3], 1);//2
    //     else if (i <= 93) gpio_set_level(laser[4], 1);//3
    //     else if (i <= 97) gpio_set_level(laser[5], 1);//4
    //     else if (i <= 100) gpio_set_level(laser[6], 1);//5
    //     else if (i <= 103) gpio_set_level(laser[7], 1);
    //     else gpio_set_level(laser[8], 1);
    // } else if (currentDuty <= -10 && currentDuty > -20) {
    //     if (i <= 82) gpio_set_level(laser[0], 1);
    //     else if (i <= 85) gpio_set_level(laser[1], 1);
    //     else if (i <= 88) gpio_set_level(laser[2], 1);//1
    //     else if (i <= 92) gpio_set_level(laser[3], 1);//2
    //     else if (i <= 95) gpio_set_level(laser[4], 1);//3
    //     else if (i <= 99) gpio_set_level(laser[5], 1);//4
    //     else if (i <= 102) gpio_set_level(laser[6], 1);//5
    //     else if (i <= 106) gpio_set_level(laser[7], 1);
    //     else gpio_set_level(laser[8], 1);
    // } else if (currentDuty >= 20) {
    //     if (i <= 79) gpio_set_level(laser[0], 1);//A
    //     else if (i <= 82) gpio_set_level(laser[1], 1);//B
    //     else if (i <= 85) gpio_set_level(laser[2], 1);//1
    //     else if (i <= 89) gpio_set_level(laser[3], 1);//2
    //     else if (i <= 92) gpio_set_level(laser[4], 1);//3
    //     else if (i <= 95) gpio_set_level(laser[5], 1);//4
    //     else if (i <= 99) gpio_set_level(laser[6], 1);//5
    //     else if (i <= 102) gpio_set_level(laser[7], 1);//C
    //     else gpio_set_level(laser[8], 1);//D
    // } else if (currentDuty <= -20) {
    //     if (i <= 84) gpio_set_level(laser[0], 1);
    //     else if (i <= 86) gpio_set_level(laser[1], 1);
    //     else if (i <= 90) gpio_set_level(laser[2], 1);//1
    //     else if (i <= 94) gpio_set_level(laser[3], 1);//2
    //     else if (i <= 97) gpio_set_level(laser[4], 1);//3
    //     else if (i <= 100) gpio_set_level(laser[5], 1);//4
    //     else if (i <= 103) gpio_set_level(laser[6], 1);//5
    //     else if (i <= 107) gpio_set_level(laser[7], 1);
    //     else gpio_set_level(laser[8], 1);
    // }
}