/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
* 2026-4            lscyzq         init
********************************************************************************************************************/
#include "zf_common_headfile.h"

int16 LCorner, RCorner, s2,centerLineCount, RUN_MODE = ZEBRA, pitch = 0, LRoadCount = expRoadCount, RRoadCount = expRoadCount;
static int16 p, i, d, s1, am, borderInTriggerDistance, ab;
uint8 borderIn, switch1, mustStop, switch2, ALONG = ALONG_R, button, disEd;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount], target;
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];
static uint8 have, data_buffer[32];

void pitInit();

void loadRoad();

uint8 haveZebra();

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    ips200_init();
    while (mt9v03x_init());
    while (imu660rb_init());
    //while (dl1b_init());
    while (wireless_uart_init());
    gpio_init(IO_PB1, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_PB0, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P32, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P65, GPO, 0, GPO_PUSH_PULL); //蜂鸣器
    gpio_init(IO_P64, GPO, 0, GPO_PUSH_PULL); //示波器
    laserInit();
    pitInit();
    displayBorderLineImage();
    while (1) {
        system_delay_ms(50);
#if (DEBUG_MODE)
        displayBinarizationImage();
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            sscanf(data_buffer, "%d,%d,%d,%d,%d,%d,%d,%d", &p, &i, &d, &am, &s1, &s2, &ab, &borderInTriggerDistance);
            switch2 = gpio_get_level(IO_PB1);
            switch1 = gpio_get_level(IO_PB0);
            ips200_clear(RGB565_WHITE);
            motorInit(p, i, d, am, s1, s2, ab);
            break;
        }
#else
        switch2 = gpio_get_level(IO_PB1);
        switch1 = gpio_get_level(IO_PB0);
        button = gpio_get_level(IO_P32);
        if (!switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "RUN_MODE: 1");
            ips200_show_string(0, MT9V03X_H + 40, "TAT    ");
        } else if (switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "RUN_MODE: 2");
            ips200_show_string(0, MT9V03X_H + 40, "^_^    ");
        } else if (!switch1 && switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "RUN_MODE: 3");
            ips200_show_string(0, MT9V03X_H + 40, "^o^     ");
        } else {
            ips200_show_string(0, MT9V03X_H + 10, "RUN_MODE: 4");
            ips200_show_string(0, MT9V03X_H + 40, "Σ(°△°)  ");
        }
        if (!button) break;
#endif
    }
    ips200_clear(RGB565_WHITE);
    ips200_show_string(0, 0, "RUNNING");
    ips200_show_string(0, 30, "(? ?_?)?");
#if (!DEBUG_MODE)
    system_delay_ms(500);
    system_delay_ms(500);
    system_delay_ms(500);
    borderInTriggerDistance = 30;
    if (!switch1 && !switch2) {
        motorInit(3000, 40, 1000, 1400, 250, 0, 0);
    } else if (switch1 && !switch2) {
        motorInit(3000, 40, 1000, 1800, 340, 0, 950);
    } else if (!switch1 && switch2) {
        motorInit(3000, 40, 1000, 2100, 370, 0, 950);
    } else {
        motorInit(3000, 40, 1000, 2500, 420, 0, 950);
    }
#endif
    if (DEBUG_MODE) RUN_MODE = FORWARD;
    system_delay_ms(500);
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, outputImage, MT9V03X_W, MT9V03X_H);
    while (1) {
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            mustStop = 1;
            break;
        }
        if (mt9v03x_finish_flag) {
            // memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            // seekfree_assistant_camera_send();
#if (DEBUG_MODE)
            if (switch1) displayBorderLineImage();
#endif
            gpio_set_level(IO_P64, 1);
            loadRoad();
            gpio_set_level(IO_P64, 0);
            if (target.y) startLaser(target.y);
            have = haveZebra();
            if (RUN_MODE == ZEBRA || ((RUN_MODE == FORWARD || RUN_MODE == BLOCK) && have)) {
                RUN_MODE = ZEBRA;
                wireless_uart_send_string("zebra\r\n");
                runZebra(have);
            }
            // else if (RUN_MODE == RAMP || (RUN_MODE == FORWARD && dl1b_distance_mm < 600
            //      && dl1b_distance_mm > 100 && !borderIn && !LCorner && !RCorner)) {
            //     RUN_MODE = RAMP;
            //     wireless_uart_send_string("ramp\r\n");
            //     runRamp();
            // }
            else if (RUN_MODE == BLOCK) {
                runBlock(LCorner, RCorner);
                wireless_uart_send_string("block\r\n");
            }
            else if (RUN_MODE == CROSS ||
                       (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2 && RRoad[0].x > 115) || (LCorner && LRoadCount > 2 && LRoad[0].x > 115)) && LRoadCount < 14 &&
                        RRoadCount < 14)) {
                RUN_MODE = CROSS;
                wireless_uart_send_string("cross\r\n");
                runCross();
            } else if (RUN_MODE == R_ROUND ||
                       (RUN_MODE == FORWARD && !borderIn && RCorner && LRoadCount > 30 && RRoadCount < 8 && RRoadCount > 3)) {
                RUN_MODE = R_ROUND;
                wireless_uart_send_string("r_round\r\n");
                runRRound(LCorner, RCorner);
            } else if (RUN_MODE == L_ROUND ||
                       (RUN_MODE == FORWARD && !borderIn && LCorner && RRoadCount > 30 && LRoadCount < 8 && LRoadCount > 3)) {
                RUN_MODE = L_ROUND;
                wireless_uart_send_string("l_round\r\n");
                runLRound(LCorner, RCorner);
            } else {
                if (centerLineCount < 10) smoothStop = 1;
                else {
                    smoothStop = 0;
                    alongLine(centerLine, centerLineCount, getDynamicAimPos());
                }
            }

// #if(DEBUG_MODE)
//             if (!disEd && RUN_MODE == BLOCK) {
//                 disEd = 1;
//                 memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
//                 for (i = 0; i < LBorderCount; i++)
//                     outputImage[LBorder[i].x][LBorder[i].y] = 0;
//                 for (i = 0; i < RBorderCount; i++)
//                     outputImage[RBorder[i].x][RBorder[i].y] = 0;
//                 ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, 0);
//                 ips200_show_int16(0, MT9V03X_H + 20, LCorner);
//                 ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
//                 ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
//                 ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
//                 ips200_show_int16(0, 300, RUN_MODE);
//             } else if (RUN_MODE == FORWARD) disEd = 0;
// #endif
            mt9v03x_finish_flag = 0;
        }
    }
}

void loadRoad() {
    int16 data i, j, pos;
    Point LStart = getLStart(), RStart = getRStart();
    target = findTarget(targetPos);
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount);
    getRBorder(RRoad, RStart, &RRoadCount);
    getGodRoadLine(LRoad, LRoadCount);
    getGodRoadLine(RRoad, RRoadCount);
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount, 0);
    RCorner = findCorner(RRoad, RRoadCount, 0);
    borderIn = 0;
    for (i = LRoadCount - 2; i >= 0 && LRoad[i].y > 92; i -= 3)
        if (LRoad[i].x > borderInTriggerDistance) {
            borderIn = 1;
            break;
        }
    if (!borderIn)
        for (i = RRoadCount - 2; i >= 0 && RRoad[i].y < 92; i -= 3)
            if (RRoad[i].x > borderInTriggerDistance) {
                borderIn = 1;
                break;
            }
    if (LCorner >= LRoadCount - 1) LCorner = 0;
    if (RCorner >= RRoadCount - 1) RCorner = 0;
    if (LCorner) {
        if (LCorner < 14 && RUN_MODE == FORWARD && LRoad[func_limit_ab(LCorner + 5, 0, LRoadCount - 1)].y - LRoad[LCorner].y > 5) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 14 && RUN_MODE == FORWARD && RRoad[RCorner].y - RRoad[func_limit_ab(RCorner + 5, 0, RRoadCount - 1)].y > 5) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_L;
            centerLineDistance = 9;
        }
        RRoadCount = RCorner;
    }
    if (ALONG == ALONG_L) {
        getCenterLineByLBorder(LRoad, centerLine, LRoadCount);
        centerLineCount = LRoadCount;
    } else {
        getCenterLineByRBorder(RRoad, centerLine, RRoadCount);
        centerLineCount = RRoadCount;
    }
    for (i = 1; i < centerLineCount; i++)
        if (centerLine[i].x < MT9V03X_H - 1) {
            i--;
            break;
        }
    centerLine[i].x = MT9V03X_H - 1;
    centerLine[i].y = centerLine[i + 1].y;
    filterRoad(centerLine, centerLineCount);
    resampleRoad(centerLine, &centerLineCount, i, 2);
    ALONG = LRoadCount > RRoadCount ? ALONG_L : ALONG_R;
#if (DEBUG_MODE)
    if (switch1) {
        for (i = 0; i < MT9V03X_H; i++)
            for (j = 0; j < MT9V03X_W; j++)
                outputImage[i][j] = 0;
        for (i = 0; i < centerLineCount; i++)
            outputImage[centerLine[i].x][centerLine[i].y] = 255;
        for (i = 0; i < LRoadCount; i++)
            outputImage[LRoad[i].x][LRoad[i].y] = 255;
        for (i = 0; i < RRoadCount; i++)
            outputImage[RRoad[i].x][RRoad[i].y] = 255;
        ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
        ips200_show_int16(0, MT9V03X_H + 20, LCorner);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
        ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
        ips200_show_int16(0, 300, RUN_MODE);
    }
#endif
}

uint8 haveZebra() {
    int16 data i = 0;
    if (!LBorderCount) return 0;
    while (i < LBorderCount && LBorder[i].x > ZEBRA_TRIGGER_POS)
        i += 3;
    if (LBorder[i].x > ZEBRA_TRIGGER_POS) return 0;
    return findZebra(i);
}

void pitHandlerImu() {
    imu660rb_get_acc();
    imu660rb_get_gyro();
}

void pitHandlerTof() {
    dl1b_get_distance();
}

void pitInit() {
    //pit_ms_init(TIM6_PIT, 10, pitHandlerTof);
    pit_ms_init(TIM0_PIT, 5, pitHandlerImu);
}