/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是
* 开源软件，在 GPL3.0 开源协议下发布
* 您可以免费使用、修改和分发本软件，但必须保留本版权声明和免责声明
*
* 本程序仅供学习交流使用，不可用于任何商业用途
* 本程序遵循 GPL3.0 开源协议
* 您应该已经收到了一份 GPL 协议的副本
* 如果没有，请访问 <https://www.gnu.org/licenses/>
*
* 文件名称：main.c
* 公司名称：逐飞科技
* 版本信息：D00
* 编写作者：W
* 完成日期：2025-11-20
* 修改作者：lscyzq
* 修改日期：2026-4
********************************************************************************************************************/
#include "zf_common_headfile.h"

// ==================== 全局变量 ====================
int16 LCorner, RCorner, s2, centerLineCount, RUN_MODE = ZEBRA, LRoadCount = expRoadCount, RRoadCount = expRoadCount;
static int16 p, i, d, s1, imBtn, borderInTriggerDistance = 25, ab;
uint8 borderIn, switch1, switch2, ALONG = ALONG_R, topBtn, bottomBtn;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount]; // 已删除 target 变量
extern uint8 outputImage[MT9V03X_H][MT9V03X_W], canNotInRamp, LThreshold, RThreshold, threshold;
static uint8 have, data_buffer[32], send_buff[32], findBorderIn;
extern int16 currentDuty;
int16 i, j, pos; // 修正了原代码中 `int16 data i, j, pos;` 的语法错误

void loadRoad();
uint8 haveZebra();

// ==================== 函数定义 ====================

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    ips200_init();
    while (mt9v03x_init());
    // while (dl1b_init()); // 已删除 TOF 测距初始化
    // pit_ms_init(TIM6_PIT, 10, tof); // 已删除 TOF 定时器中断，纯循迹无需测距
    
#if (DEBUG_MODE)
    while (wireless_uart_init());
#endif
    gpio_init(IO_PB1, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_PB0, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P02, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P01, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P32, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P90, GPO, 0, GPO_PUSH_PULL); 
    // gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); // 已删除激光使能引脚
    // laserInit(); // 已删除激光初始化
    
    LThreshold = RThreshold = threshold = mt9v03x_image[80][94] + mt9v03x_image[107][94] >> 1; 
    //所以车一开机，它会先看看车头正前方的地面，默认这里是白色的赛道。
    然后记住这个亮度，把这个亮度作为“标准值”，去判断整张图片。比这个值亮的是白赛道，暗的是黑边线。这叫动态阈值。
    //发车的时候，千万要把车放在白色的赛道上，别放在黑线（路障、斑马线）上。
    
    displayBorderLineImage();
    //把车“看”到的画面，画到你的 IPS 屏幕上
    
    while (1) {
        system_delay_ms(50);
#if (DEBUG_MODE)  //电脑的无线串口调试
        if (wireless_uart_read_buffer(data_buffer, 32)) {   
            sscanf(data_buffer, "%d,%d,%d,%d,%d,%d", &p, &i, &d, &s1, &s2, &ab);  //在电脑上写下数据，读取电脑发送的数据
            switch2 = gpio_get_level(IO_PB1);   //读取拨码开关的状态，为下面的做准备
            switch1 = gpio_get_level(IO_PB0);
            ips200_clear(RGB565_WHITE);  //清屏
            motorInit(p, i, d, s1, s2, ab, 0);  //设置参数
            break;
        }
#else   //这段不是上面的电脑调参，是物理调参
        switch2 = gpio_get_level(IO_PB1);   //下面的四个按键组合成8个速度选择
        switch1 = gpio_get_level(IO_PB0);
        topBtn = gpio_get_level(IO_P02);
        bottomBtn = gpio_get_level(IO_P01);
        imBtn = gpio_get_level(IO_P32);  //图像查看按键
        if (!imBtn) {     //图像查看，它把摄像头拍到的赛道、车算出来的左边界、右边界、中线全都画在屏幕上给你看。
                            并且屏幕上还会显示 LRoadCount（左边界找了多少点）、LCorner（左边是不是有拐角）等等数据。
                             你按了这个键，车就不跑了，你可以对着屏幕，看看车“眼里”的赛道长啥样，判断车找线找得对不对。
            while (1) {
                if (mt9v03x_finish_flag) {    //照了一个图像
                    loadRoad();      //对图像做处理
                    displayBinarizationImage();   //把识别到的中线、边线画到一张空白图上
                    for (i = 0; i < MT9V03X_H; i++)
                        for (j = 0; j < MT9V03X_W; j++)
                            outputImage[i][j] = 0;
                    for (i = 0; i < centerLineCount; i++)
                        outputImage[centerLine[i].x][centerLine[i].y] = 255;
                    for (i = 0; i < LRoadCount; i++)
                        outputImage[LRoad[i].x][LRoad[i].y] = 255;
                    for (i = 0; i < RRoadCount; i++)
                        outputImage[RRoad[i].x][RRoad[i].y] = 255;
                    ips200_displayimage03x(outputImage, MT9V03X_W, MT9V03X_H, 0, MT9V03X_H + 40);  //显示到屏幕上
                    ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);    //把边线数量、拐角状态显示在屏幕上
                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
                    ips200_show_int16(0, MT9V03X_H + 20, LCorner);
                    ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
                    ips200_show_int16(0, 300, RUN_MODE);
                    mt9v03x_finish_flag = 0;
                }
            }
        }
        if (!switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 1");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 2");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 1");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 26, 4500, 520, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 2");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(200, 800, 22, 4000, 300, 0, 0);
                break;
            }
        } else if (switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 3");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 4");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 3");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(210, 900, 23, 4000, 320, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 4");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(220, 1000, 23, 4000, 340, 0, 0);
                break;
            }
        } else if (!switch1 && switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 5");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 6");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 5");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(230, 1000, 24, 4000, 360, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 6");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(240, 1000, 24, 4000, 380, 0, 0);
                break;
            }
        } else {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 7");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 8");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 7");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(250, 1000, 25, 4000, 450, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 8");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 25, 4000, 500, 0, 0);
                break;
            }
        }
#endif
    }
    // dismissLaser(); // 已删除关闭激光操作
    imuInit();
    if (DEBUG_MODE) RUN_MODE = FORWARD;
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, outputImage, MT9V03X_W, MT9V03X_H);
    
    while (1) {
#if (DEBUG_MODE)
        if (wireless_uart_read_buffer(data_buffer, 32)) {
            stop = 1;
            gpio_set_level(IO_P90, 0);
            break;
        }
#endif
        if (RUN_MODE == RAMP || (!canNotInRamp && (RUN_MODE == FORWARD || RUN_MODE == BLOCK) && pitch > 7)) {
            RUN_MODE = RAMP;
            //wireless_uart_send_string("RP¥n");
            runRamp();
        }
        if (mt9v03x_finish_flag) {
            // memcpy(outputImage, mt9v03x_image, MT9V03X_IMAGE_SIZE);
            // seekfree_assistant_camera_send();
#if (DEBUG_MODE)
            if (switch1) displayBorderLineImage();
#endif
            //gpio_set_level(IO_P04, 1);
            loadRoad();
            // gpio_set_level(IO_P04, 0);
            // if (target.y) startLaser(target.y); // 已删除打靶触发逻辑
            
            have = haveZebra();
            if (RUN_MODE == ZEBRA || have) {
                RUN_MODE = ZEBRA;
                //wireless_uart_send_string("ZB¥n");
                // sprintf(send_buff, "ZEBRA: %d,%d¥n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
                runZebra(have);
            } else if (RUN_MODE == BLOCK) {
                runBlock(LCorner, RCorner);
                // sprintf(send_buff, "BLOCK: %d,%d¥n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else if (RUN_MODE == CROSS ||
                       (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2) || (LCorner && LRoadCount > 2)) && LRoadCount < 14 &&
                        RRoadCount < 14)) {
                RUN_MODE = CROSS;
                // sprintf(send_buff, "CROSS: %d,%d¥n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
                runCross();
            } else if (RUN_MODE == R_ROUND ||
                       (RUN_MODE == FORWARD && LRoad[37].x < 20 && RCorner && LRoadCount > 37 && RRoadCount < 11)) {
                RUN_MODE = R_ROUND;
                borderIn = 1;
                //wireless_uart_send_string("RR¥n");
                runRRound(LCorner, RCorner);
                // sprintf(send_buff, "R_ROUND: %d,%d¥n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else if (RUN_MODE == L_ROUND ||
                       (RUN_MODE == FORWARD && RRoad[37].x < 20 && LCorner && RRoadCount > 37 && LRoadCount < 11)) {
                RUN_MODE = L_ROUND;
                borderIn = 1;
                runLRound(LCorner, RCorner);
                //wireless_uart_send_string("LR¥n");
                // sprintf(send_buff, "L_ROUND: %d,%d¥n", LRoadCount, RRoadCount);
                // wireless_uart_send_buffer(send_buff, 32);
            } else
                alongLine(centerLine, centerLineCount, getAimPos());
            mt9v03x_finish_flag = 0;
        }
    }
}

// ==================== 已删除的打靶函数 ====================
// int8 DTarget(int16 speed) { ... } 已删除
// int8 DTargetR(int16 speed) { ... } 已删除
// void findTarget() { ... } 已删除

void loadRoad() {
    Point LStart = getLStart(), RStart = getRStart();
    // findTarget(); // 已删除靶标识别调用
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount, 0);
    getRBorder(RRoad, RStart, &RRoadCount, 0);
    LBorderCount = LRoadCount;
    memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
    RBorderCount = RRoadCount;
    memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
    getGodRoadLine(LRoad, LRoadCount);
    getGodRoadLine(RRoad, RRoadCount);
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount, 0);
    RCorner = findCorner(RRoad, RRoadCount, 0);
    findBorderIn = 0;
    for (i = 0; i < LRoadCount; i += 2)
        if (LRoad[i].y > 94) {
            findBorderIn = 1;
            break;
        }
    if (LRoad[i].x > borderInTriggerDistance && findBorderIn) borderIn = 1;
    else {
        for (i = 0; i < RRoadCount; i += 2)
            if (RRoad[i].y < 94) {
                findBorderIn = 1;
                break;
            }
        if (RRoad[i].x > borderInTriggerDistance && findBorderIn) borderIn = 1;
        else borderIn = 0;
    }
    if (LCorner) {
        if (LCorner < 3) LCorner = 0;
        else if (LCorner < 15 && RUN_MODE == FORWARD && LRoad[func_limit_ab(LCorner + 3, 0, LRoadCount - 1)].y > LRoad[LCorner].y) {
            RUN_MODE = BLOCK;
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 3) RCorner = 0;
        else if (RCorner < 15 && RUN_MODE == FORWARD && RRoad[RCorner].y > RRoad[func_limit_ab(RCorner + 3, 0, RRoadCount - 1)].y) {
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
        if (centerLine[i].x < 119) {
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
        ips200_show_int16(0, MT9V03X_H + 1, LRoadCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RRoadCount);
        ips200_show_int16(0, MT9V03X_H + 20, LCorner);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 20, RCorner);
        ips200_show_int16(0, 300, RUN_MODE);
    }
#endif
}

uint8 haveZebra() {
    int16 i = 0;
    if (!LBorderCount) return 0;
    while (i < LBorderCount && LBorder[i].x > ZEBRA_TRIGGER_POS)
        i += 3;
    if (LBorder[i].x > ZEBRA_TRIGGER_POS) return 0;
    return findZebra(i);
}
