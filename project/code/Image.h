//
// Created by 31663 on 2026/3/16.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#include "zf_common_headfile.h"

#define block 5               //¾ÖÓò¶þÖµ»¯±ß³¤     //局部二值化边长的参数（决定怎么把灰度图变成黑白图）
#define cap 6                //È¥Ôë   5            //去噪参数（把图像上的小噪点抹掉，代码里写了注释“去噪 5”）
#define expRoadCount 100      //±ßÏßÆÚÍû³¤¶È       //边线期望长度。意思是车准备用 100 个点来记录左边线或右边线。
#define ZEBRA_TRIGGER_POS 40                       //斑马线触发位置。车要跑多远才开始仔细看斑马线。
#define targetPos 78

void getLBorder(Point LRoad[], Point start, int16 *count, int8 delicate);         //找左边线
void getRBorder(Point RRoad[], Point start, int16 *count, int8 delicate);
void getCenterLineByLBorder(Point roadLine[], Point centerLine[], int16 count);    //根据左边线，算中线
void getCenterLineByRBorder(Point roadLine[], Point centerLine[], int16 count);
void filterRoad(Point rodeLine[], int16 count);                //对找到的线进行“磨皮滤波”和“均匀取点”（修图工具）
void getGodRoadLine(Point roadLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();           //找左边线和右边线的起点
Point getRStart();
int16 findCorner(Point roadLine[], int16 count, int8 cross);      //找拐角（判断是不是急弯）
uint8 findZebra(int16 pos);        //找斑马线
Point findTargetByL(int16 i);
extern const uint8 borderOffset[120];         //畸变补偿表。摄像头是凸透镜，拍出来的直线其实是弯的。这个数组是“眼镜片”，把弯的拉直。
uint8 getImagePointFilterSelf(uint8 i, uint8 j);         //在图像里，去读取第 i 行、第 j 列的像素点，并且带自我过滤。
extern uint8 threshold, centerLineDistance, banUpdateThreshold;     
//threshold：动态阈值。车看黑白的分界线。centerLineDistance：中心线偏移距离。平时它是0，车沿着正中心线走。
//但当你遇到路障（BLOCK）时，你在代码里写了 centerLineDistance = 9;，意思是让车强行偏离赛道中线9个像素，去绕开障碍物。
//banUpdateThreshold：“禁止更新阈值”。这是个保护机制。车在跑的时候，阈值会根据光线自动微调。
//但过斑马线或者上坡时，路面全黑或全白，如果这时候还自动微调，车就瞎了。所以这个变量平时是0（允许更新），到了特殊赛道元素，把它设为1（锁死光线阈值，保持不变）
extern int16 LBorderCount, RBorderCount;           //记录左边线和右边线分别找到了多少个点       
extern Point LBorder[expRoadCount], RBorder[expRoadCount];         //左边线找到第1个点，就存进 LBorder[0]；找到第2个点，存进 LBorder[1]
void displayBinarizationImage();   //显示黑白二值化图   //这些函数专门用来在屏幕上画图的，你按下图像查看按键（imBtn）时，屏幕显示的就是它们的功劳
void displayBorderLineImage();     //显示边线图
void displayGodImage();             //显示“上帝视角”（俯视图）  
void displayGodRoadLineImage();    

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
