//
// Created by 31663 on 2026/3/16.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#include "zf_common_headfile.h"

#define block 5               //局域二值化边长
#define cap 5                 //去噪
#define expRoadCount 170      //边线期望长度
#define expThreshold 150

void getLBorder(Point LBorder[], Point start, int16 *count);
void getRBorder(Point RBorder[], Point start, int16 *count);
void getCenterLineByLBorder(Point roadLine[], Point centerLine[], int16 count);
void getCenterLineByRBorder(Point roadLine[], Point centerLine[], int16 count);
void filterRoad(Point rodeLine[], int16 count);
void getGodRoadLine(Point roadLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();
Point getRStart();
int16 findCorner(Point roadLine[], int16 count);
extern uint8 threshold, centerLineDistance;
void displayBinarizationImage();
void displayBorderLineImage();
void displayGodImage();
void displayGodRoadLineImage();
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
