//
// Created by 31663 on 2026/7/3.
//

#include "Laser.h"

gpio_pin_enum laser[] = {IO_P17, IO_P67, IO_PB2, IO_P44, IO_PB5, IO_PB3, IO_PB4, IO_P45, IO_P65, IO_P14, IO_P91, IO_P16};
void laserInit() {
    int8 i;
    for (i = 0; i < 12; i++)
        gpio_init(laser[i], GPO, 0, GPO_PUSH_PULL);
}

void startLaser(uint8 i) {
    uint8 j;
    for (j = 0; j < 12; j++)
        gpio_set_level(laser[j], 0);
    if (i <= 85) return;
    if (i <= 88) gpio_set_level(laser[2], 1);
    else if (i <= 92) gpio_set_level(laser[3], 1);
    else if (i <= 96) gpio_set_level(laser[4], 1);
    else if (i <= 99) gpio_set_level(laser[5], 1);
    else if (i <= 102) gpio_set_level(laser[6], 1);
}