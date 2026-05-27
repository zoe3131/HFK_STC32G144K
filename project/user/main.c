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

int16 LCorner, RCorner, s2,centerLineCount, RUN_MODE = FORWARD, pitch = 0, LRoadCount = expRoadCount, RRoadCount = expRoadCount;
static int16 p, i, d, s1, am, cnm;
uint8 borderIn, display, mustStop, crossDis, ALONG = ALONG_R, dised;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
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
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, outputImage, MT9V03X_W, MT9V03X_H);
    while (1) {
        if (mt9v03x_finish_flag) {
            // memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            // seekfree_assistant_camera_send();
            if (!display) displayBorderLineImage();
            //gpio_set_level(IO_P77, 1);
            loadRoad();
            have = haveZebra();
            if (RUN_MODE == ZEBRA || ((RUN_MODE == FORWARD || RUN_MODE == BLOCK) && have)) {
                RUN_MODE = ZEBRA;
                runZebra(have);
                wireless_uart_send_string("z\r\n");
            } else if (RUN_MODE == RAMP || (RUN_MODE == FORWARD && dl1b_distance_mm < 350 && dl1b_distance_mm > 100 && !borderIn)) {
                RUN_MODE = RAMP;
                runRamp();
                wireless_uart_send_string("r\r\n");
            } else if (RUN_MODE == BLOCK) {
                runBlock(LCorner, RCorner);
                wireless_uart_send_string("b\r\n");
            } else if (RUN_MODE == CROSS ||
                       (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2) || (LCorner && LRoadCount > 2)) && LRoadCount < 12 &&
                        RRoadCount < 12)) {
                RUN_MODE = CROSS;
                runCross();
                wireless_uart_send_string("c\r\n");
            } else if (RUN_MODE == R_ROUND ||
                       (RUN_MODE == FORWARD && RCorner && LRoadCount > 30 && RRoadCount < 8 && RRoadCount > 3)) {
                RUN_MODE = R_ROUND;
                runRRound(LCorner, RCorner);
                wireless_uart_send_string("rr\r\n");
            } else if (RUN_MODE == L_ROUND ||
                       (RUN_MODE == FORWARD && LCorner && RRoadCount > 30 && LRoadCount < 8 && LRoadCount > 3)) {
                RUN_MODE = L_ROUND;
                runLRound(LCorner, RCorner);
                wireless_uart_send_string("lr\r\n");
            } else {
                if (centerLineCount < 10) smoothStop = 1;
                else {
                    smoothStop = 0;
                    alongLine(centerLine, centerLineCount, getDynamicAimPos());
                }
            }
            mt9v03x_finish_flag = 0;
        }
    }
}

void loadRoad() {
    int16 i, j, pos;
    Point LStart = getLStart(), RStart = getRStart();
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount);
    getRBorder(RRoad, RStart, &RRoadCount);
    getGodRoadLine(LRoad, LRoadCount);
    getGodRoadLine(RRoad, RRoadCount);
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount);
    RCorner = findCorner(RRoad, RRoadCount);
    borderIn = 0;
    for (i = LRoadCount - 2; i >= 0 && LRoad[i].y > 92; i -= 2)
        if (LRoad[i].x > cnm) {
            borderIn = 1;
            break;
        }
    if (!borderIn)
        for (i = RRoadCount - 2; i >= 0 && RRoad[i].y < 92; i -= 2)
            if (RRoad[i].x > cnm) {
                borderIn = 1;
                break;
            }
    if (LCorner >= LRoadCount - 1) LCorner = 0;
    if (RCorner >= RRoadCount - 1) RCorner = 0;
    if (LCorner) {
        if (LCorner < 20 && RUN_MODE == FORWARD && LRoad[func_limit_ab(LCorner + 5, 0, LRoadCount - 1)].y - LRoad[LCorner].y > 4) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 20 && RUN_MODE == FORWARD && RRoad[RCorner].y - RRoad[func_limit_ab(RCorner + 5, 0, RRoadCount - 1)].y > 4) {
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
    if (!display) {
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
        ips200_show_int16(0, MT9V03X_H + 20, LRoad[LCorner].x);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RRoad[RCorner].x);
        ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
        ips200_show_int16(0, 300, RUN_MODE);
    }
}

uint8 haveZebra() {
    int16 pos = 0;
    if (LBorderCount < RBorderCount) {
        while (pos < LBorderCount && LBorder[pos].x > ZEBRA_TRIGGER_POS)
            pos += 3;
        pos = func_min(pos, LBorderCount - 1);
        return pos <= 0 || LBorder[pos].x > 97 ? 0 : isZebra(pos, ALONG_L);
    }
    while (pos < RBorderCount && RBorder[pos].x > ZEBRA_TRIGGER_POS)
        pos += 3;
    pos = func_min(pos, RBorderCount - 1);
    return pos <= 0 || RBorder[pos].x > 97 ? 0 : isZebra(pos, ALONG_R);
}

void pitHandlerImu() {
    imu660rb_get_acc();
    imu660rb_get_gyro();
}

void pitHandlerTof() {
    dl1b_get_distance();
}

void pitInit() {
    pit_ms_init(TIM6_PIT, 10, pitHandlerTof);
    pit_ms_init(TIM0_PIT, 5, pitHandlerImu);
}