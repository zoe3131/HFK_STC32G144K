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
********************************************************************************************************************/
#include "zf_common_headfile.h"

#define WIFI_SSID "REDMI K80 Pro"
#define WIFI_PASSWD "abhgy789"
int16 LCorner, RCorner, s2,centerLineCount, RUN_MODE = FORWARD, pitch = 0, ALONG = ALONG_R, LRoadCount = expRoadCount, RRoadCount = expRoadCount;
static int16 p, i, d, s1, am, cnm;
uint8 borderIn, display, mustStop, crossDis;
Point centerLine[expRoadCount], LBorder[expRoadCount], RBorder[expRoadCount];
extern uint8 outputImage[MT9V03X_H][MT9V03X_W];
static uint8 have, data_buffer[32];

void pitInit();

void loadRoad();

uint8 haveZebra();

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                            // 调试串口信息初始化
    ips200_init();
    while (mt9v03x_init());
    while (imu660rb_init());
    while (dl1b_init());
    while (wireless_uart_init());
    gpio_init(IO_PB1, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P64, GPO, 1, GPO_PUSH_PULL);
    gpio_init(IO_PB0, GPI, 1, GPI_PULL_UP);
    gpio_init(IO_P65, GPO, 0, GPO_PUSH_PULL); //蜂鸣器
//    //gpio_init(IO_P77, GPO, 0, GPO_PUSH_PULL); //示波器
    pitInit();
    ips200_show_string(0, MT9V03X_H + 20, "READY");
    displayBorderLineImage();
    while (1) {
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            sscanf(data_buffer, "%d,%d,%d,%d,%d,%d,%d", &p, &i, &d, &am, &s1, &s2, &cnm);
            display = gpio_get_level(IO_PB1);
            crossDis = gpio_get_level(IO_PB0);
            ips200_clear(RGB565_WHITE);
            motorInit(p, i, d, am, s1, s2);
            break;
        }
        system_delay_ms(50);
    }
    system_delay_ms(3000);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, outputImage, MT9V03X_W, MT9V03X_H);
    while (1) {
        if (mt9v03x_finish_flag) {
            memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            seekfree_assistant_camera_send();
            if (!display) displayBorderLineImage();
            have = haveZebra();
            //ips200_show_int16(0, 15, imu660rb_gyro_x);
            //gpio_set_level(IO_P77, 1);
            loadRoad();
//            if (RUN_MODE == ZEBRA || (RUN_MODE == FORWARD && have)) {
//                RUN_MODE = ZEBRA;
//                runZebra(have);
//            } else
            if (RUN_MODE == RAMP || (RUN_MODE == FORWARD && dl1b_distance_mm < 350 && dl1b_distance_mm > 100 && !borderIn)) {
                RUN_MODE = RAMP;
                runRamp();
            }
            // else if (RUN_MODE == BLOCK) {
            //     runBlock(LCorner, RCorner);
            // }
            else if (RUN_MODE == CROSS ||
                       (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2) || (LCorner && LRoadCount > 2)) && LRoadCount < 12 &&
                        RRoadCount < 12)) {
                RUN_MODE = CROSS;
                runCross();
            } else if (RUN_MODE == R_ROUND ||
                       (RUN_MODE == FORWARD && RCorner && LRoadCount > 30 && RRoadCount < 8 && RRoadCount > 3)) {
                RUN_MODE = R_ROUND;
                runRRound(LCorner, RCorner);
            } else if (RUN_MODE == L_ROUND ||
                       (RUN_MODE == FORWARD && LCorner && RRoadCount > 30 && LRoadCount < 8 && LRoadCount > 3)) {
                RUN_MODE = L_ROUND;
                runLRound(LCorner, RCorner);
            } else {
                if (centerLineCount < 10) smoothStop = 1;
                else {
                    smoothStop = 0;
                    alongLine(centerLine, centerLineCount, getDynamicAimPos());
                }
            }
//            if (RUN_MODE != FORWARD) {
//                if (!dised) {
//                    int16 i, j;
//                    dised = 1;
//                    displayBorderLineImage();
//                    for (i = 0; i < MT9V03X_H; i++)
//                        for (j = 0; j < MT9V03X_W; j++)
//                            outputImage[i][j] = 0;
//                    for (i = 0; i < centerLineCount; i++)
//                        outputImage[centerLine[i].x][centerLine[i].y] = 255;
//                    for (i = 0; i < LRoadCount; i++)
//                        outputImage[LBorder[i].x][LBorder[i].y] = 255;
//                    for (i = 0; i < RRoadCount; i++)
//                        outputImage[RBorder[i].x][RBorder[i].y] = 255;
//                    ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
//                    ips200_show_int16(0, MT9V03X_H + 20, LCorner);
//                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
//                    ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
//                    ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
//                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
//                    ips200_show_int16(0, 300, RUN_MODE);
//                }
//            } else dised = 0;
            mt9v03x_finish_flag = 0;
        }
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            sscanf(data_buffer, "%d,%d,%d,%d,%d,%d,%d", &p, &i, &d, &am, &s1, &s2, &cnm);
            motorInit(p, i, d, am, s1, s2);
//            if (mustStop) {
//                mustStop = 0;
//            } else mustStop = 1;
        }
    }
}

void loadRoad() {
    int16 i, j, pos;
    Point LStart = getLStart(), RStart = getRStart();
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LBorder, LStart, &LRoadCount);
    getRBorder(RBorder, RStart, &RRoadCount);
    getGodRoadLine(LBorder, LRoadCount);
    getGodRoadLine(RBorder, RRoadCount);
    filterRoad(LBorder, LRoadCount);
    filterRoad(RBorder, RRoadCount);
    resampleRoad(LBorder, &LRoadCount, 0, 3);
    resampleRoad(RBorder, &RRoadCount, 0, 3);
    LCorner = findCorner(LBorder, LRoadCount);
    RCorner = findCorner(RBorder, RRoadCount);
    borderIn = 0;
    for (i = LRoadCount - 2; i >= 0 && LBorder[i].y > 92; i -= 2)
        if (LBorder[i].x > cnm) {
            borderIn = 1;
            break;
        }
    if (!borderIn)
        for (i = RRoadCount - 2; i >= 0 && RBorder[i].y < 92; i -= 2)
            if (RBorder[i].x > cnm) {
                borderIn = 1;
                break;
            }
    if (LCorner >= LRoadCount - 1) LCorner = 0;
    if (RCorner >= RRoadCount - 1) RCorner = 0;
    if (LCorner) {
        if (LCorner < 20 && RUN_MODE == FORWARD && LBorder[func_limit_ab(LCorner + 5, 0, LRoadCount - 1)].y - LBorder[LCorner].y > 4) {
            gpio_set_level(IO_P65, 0);
            RUN_MODE = BLOCK;
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 20 && RUN_MODE == FORWARD && RBorder[RCorner].y - RBorder[func_limit_ab(RCorner + 5, 0, RRoadCount - 1)].y > 4) {
            gpio_set_level(IO_P65, 0);
            RUN_MODE = BLOCK;
            ALONG = ALONG_L;
            centerLineDistance = 9;
        }
        RRoadCount = RCorner;
    }
    if (ALONG == ALONG_L) {
        getCenterLineByLBorder(LBorder, centerLine, LRoadCount);
        centerLineCount = LRoadCount;
    } else {
        getCenterLineByRBorder(RBorder, centerLine, RRoadCount);
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
    if (!display) {
        for (i = 0; i < MT9V03X_H; i++)
            for (j = 0; j < MT9V03X_W; j++)
                outputImage[i][j] = 0;
        for (i = 0; i < centerLineCount; i++)
            outputImage[centerLine[i].x][centerLine[i].y] = 255;
        for (i = 0; i < LRoadCount; i++)
            outputImage[LBorder[i].x][LBorder[i].y] = 255;
        for (i = 0; i < RRoadCount; i++)
            outputImage[RBorder[i].x][RBorder[i].y] = 255;
        ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);
        ips200_show_int16(0, MT9V03X_H + 20, LBorder[LCorner].x);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RBorder[RCorner].x);
        ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
        ips200_show_int16(0, 300, RUN_MODE);
    }
}

uint8 haveZebra() {
    int16 i = 10, count = 0, j;
    int8 add = 0;
    for (j = 40; j <= 140; j += 3) {
        if (mt9v03x_image[i][j] < threshold) {
            if (add) {
                count++;
                if (count > 5) return 1;
                add = 0;
            }
        } else add = 1;
    }
    i = 20;
    count = 0;
    add = 0;
    for (j = 30; j <= 150; j += 3) {
        if (mt9v03x_image[i][j] < threshold) {
            if (add) {
                count++;
                if (count > 5) return 1;
                add = 0;
            }
        } else add = 1;
    }
    return 0;
}

void pitHandlerImu() {
    imu660rb_get_acc();
    imu660rb_get_gyro();
}

void pitHandlerTof() {
    dl1b_get_distance();
}

void pitInit() {
    pit_ms_init(TIM6_PIT, 11, pitHandlerTof);
    pit_ms_init(TIM0_PIT, 5, pitHandlerImu);
}