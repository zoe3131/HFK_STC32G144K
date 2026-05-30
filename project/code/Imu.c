//
// Created by 31663 on 2026/5/30.
//

#include "Imu.h"
int16 q8_mul(int16 a, int16 b) {
    int16 tmp = a * b;
    return (tmp + 0x80) >> 8;
}
int16 q8_invSqrt(int16 x) {
    if (x <= 0) return 0x7FFF;
    int32 s = sqrt_int(x);
    if (s == 0) return 0x7FFF;
    int32 inv = 4096 / s;
    return (int16)inv;
}
void vector_norm_q8(int16 *x, int16 *y, int16 *z) {
    int32 sq = (int32)*x**x + (int32)*y**y + (int32)*z**z;
    sq = (sq + 0x80) >> 8;
    int16 inv = q8_invSqrt((int16)sq);
    *x = q8_mul(*x, inv);
    *y = q8_mul(*y, inv);
    *z = q8_mul(*z, inv);
}
typedef struct {
    int16 w, x, y, z;        // 四元数，Q8.8格式
    int16 twoKp;             // 2*Kp，Q8.8
    int16 twoKi;             // 2*Ki，Q8.8
    int32 integralFBx;       // 积分项，Q8.8（用32位防溢出）
    int32 integralFBy;
    int32 integralFBz;
    int16 halfT;             // 采样周期的一半 dt/2，Q8.8
} Mahony6Axis_Q88;

void mahony6_init_q88(Mahony6Axis_Q88 *filter, float sampleFreq, float Kp, float Ki) {
    filter->w = 1 << 8;        // 1.0 in Q8.8
    filter->x = 0;
    filter->y = 0;
    filter->z = 0;
    filter->twoKp = (int16)(2.0f * Kp * 256.0f + 0.5f);
    filter->twoKi = (int16)(2.0f * Ki * 256.0f + 0.5f);
    filter->integralFBx = 0;
    filter->integralFBy = 0;
    filter->integralFBz = 0;
    float halfT_f = 0.5f / sampleFreq;   // dt/2
    filter->halfT = (int16)(halfT_f * 256.0f + 0.5f);
}
void mahony6_update_q88(Mahony6Axis_Q88 *filter,
                        int16 gx, int16 gy, int16 gz,
                        int16 ax, int16 ay, int16 az) {
    vector_norm_q8(&ax, &ay, &az);
    int16 q0 = filter->w, q1 = filter->x, q2 = filter->y, q3 = filter->z;
    int16 t1 = q8_mul(q1, q3);
    int16 t2 = q8_mul(q0, q2);
    int16 g_ref_x = (t1 - t2) << 1;   // 乘以2，注意可能溢出，用32位
    int32 g_ref_x32 = (int32)q1*q3 - (int32)q0*q2;
    g_ref_x = (int16)((g_ref_x32 * 2) >> 8);
    int32 g_ref_y32 = (int32)q0*q1 + (int32)q2*q3;
    int16 g_ref_y = (int16)((g_ref_y32 * 2) >> 8);
    int32 g_ref_z32 = (int32)q0*q0 - (int32)q1*q1 - (int32)q2*q2 + (int32)q3*q3;
    int16 g_ref_z = (int16)(g_ref_z32 >> 8);
    int32 err_x = (int32)ay * g_ref_z - (int32)az * g_ref_y;
    int32 err_y = (int32)az * g_ref_x - (int32)ax * g_ref_z;
    int32 err_z = (int32)ax * g_ref_y - (int32)ay * g_ref_x;
    int16 err_x_q8 = (int16)(err_x >> 8);
    int16 err_y_q8 = (int16)(err_y >> 8);
    int16 err_z_q8 = (int16)(err_z >> 8);
    if (filter->twoKi > 0) {
        int32 ki_err_x = ((int32)filter->twoKi * err_x_q8) >> 8;
        int32 ki_err_y = ((int32)filter->twoKi * err_y_q8) >> 8;
        int32 ki_err_z = ((int32)filter->twoKi * err_z_q8) >> 8;
        int32 dt = (int32)filter->halfT * 2;
        filter->integralFBx += (ki_err_x * dt) >> 8;
        filter->integralFBy += (ki_err_y * dt) >> 8;
        filter->integralFBz += (ki_err_z * dt) >> 8;
        int32 pgain = (int32)filter->twoKp * err_x_q8;
        gx += (int16)((pgain >> 8) + (filter->integralFBx >> 8));
        gy += (int16)(((int32)filter->twoKp * err_y_q8 >> 8) + (filter->integralFBy >> 8));
        gz += (int16)(((int32)filter->twoKp * err_z_q8 >> 8) + (filter->integralFBz >> 8));
    } else {
        gx += (int16)(((int32)filter->twoKp * err_x_q8) >> 8);
        gy += (int16)(((int32)filter->twoKp * err_y_q8) >> 8);
        gz += (int16)(((int32)filter->twoKp * err_z_q8) >> 8);
    }
    int16 halfT = filter->halfT;
    int32 q0_dot = - (int32)q1*gx - (int32)q2*gy - (int32)q3*gz;
    q0_dot = q0_dot >> 9;
    int32 q1_dot = ( (int32)q0*gx + (int32)q2*gz - (int32)q3*gy ) >> 9;
    int32 q2_dot = ( (int32)q0*gy - (int32)q1*gz + (int32)q3*gx ) >> 9;
    int32 q3_dot = ( (int32)q0*gz + (int32)q1*gy - (int32)q2*gx ) >> 9;
    q0 += (int16)((q0_dot * halfT) >> 8);
    q1 += (int16)((q1_dot * halfT) >> 8);
    q2 += (int16)((q2_dot * halfT) >> 8);
    q3 += (int16)((q3_dot * halfT) >> 8);
    int32 norm_sq = (int32)q0*q0 + (int32)q1*q1 + (int32)q2*q2 + (int32)q3*q3;
    norm_sq = (norm_sq + 0x80) >> 8;
    int16 inv_norm = q8_invSqrt((int16)norm_sq);
    filter->w = q8_mul(q0, inv_norm);
    filter->x = q8_mul(q1, inv_norm);
    filter->y = q8_mul(q2, inv_norm);
    filter->z = q8_mul(q3, inv_norm);
}