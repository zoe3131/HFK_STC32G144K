//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#define DEBUG_MODE 0         //这里现在是 0，意味着“比赛模式”。车开机后不看电脑脸色，直接用按键和拨码开关选速度。
#define FORWARD 114          //直行         //我说要“直行”，就给芯片发 114，我说要“过环岛”，就给芯片发 1919
#define CROSS 514            //十字
#define R_ROUND 1919         //右转
#define L_ROUND 810          //左转
#define ZEBRA 520            //斑马线
#define RAMP 1314            //坡道
#define BLOCK 521            //障碍物
#define ALONG_L 88           //选择跟哪条线走，只用于左环岛和右环岛的进环/出环逻辑
#define ALONG_R 99

volatile extern int16 setSpeed, handle, encoderL, encoderR;         //你希望车跑多快，舵机打角量，左右轮子上编码器实时测出来的速度
void motorInit(int16 c11, int16 c22, int16 c33, int16 c44, int16 c55, int16 c66, int16 c77);
void setServoDuty(int8 duty);         //这是直接控制舵机打角的函数。传进去一个数值 duty，舵机就转到对应角度。通常是在 alongLine 内部被调用，你平时不用管它
void alongLine(Point roadLine[], int16 count, int16 aimPos);         //它接收三个数据：摄像头算出来的中线数组（roadLine）、点的个数（count）、以及车应该看向中线的哪个位置（aimPos）。它内部会算出一个舵机打角量（handle），把车掰回中线上。如果状态是普通直行，main.c 最后一步就会落到这个函数里
int8 getAimPos();         //这是“眼睛的焦点”。车不能只看车头正前方 10 厘米的地面，那样速度一快根本来不及打方向盘。这个函数负责计算：“根据我现在的速度，我的眼睛应该看中线上哪个点（前瞻点），才能稳稳过弯”
extern int16 enableDifSpeed, currentSpeed;         //currentSpeed：车当前的实际平均速度。enableDifSpeed：电子差速开关。过弯时，让内轮转慢一点、外轮转快一点，防止车子推头或者甩尾。
extern int8 stop;         //紧急停车标志。在蓝牙调试模式下，你发个数据过去，这个变量变成 1，车就断电停车了
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
