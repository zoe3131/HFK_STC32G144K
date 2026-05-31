//
// Created by 31663 on 2026/3/27.
//
#include "Motor.h"

#define pressPwmL PWMF_CH2_PA3
#define pressPwmR PWMF_CH1_PA1
#define motorDirR IO_P77
#define motorPwmR PWMB_CH3_P76
#define motorDirL IO_P75
#define motorPwmL PWMB_CH1_P74

int16 encoderL, encoderR, setSpeed = 0, handle = 0, duty = 0;
static uint32 K;
const int16 tani[31];
extern int16 RUN_MODE;
extern uint8 display, borderIn, mustStop;
static uint8 DEVICE_H = 30;
static int16 R, c1, c2, c3, c4, c5, c6;
int8 smoothStop;

typedef struct {
    int16 Kp;
    int16 Ki;
    int16 Kd;
    int16 attitude_coeff;
    uint8 which;
    int16 last_error;
    int16 llast_error;
    int16 base;
    int16 max_out;
} PID_t;

PID_t steerPid, motorLPID, motorRPID;

void PIDInit(PID_t *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 attitude_coeff,
              int16 max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->last_error = 0;
    pid->llast_error = 0;
    pid->base = 0;
    pid->which = which;
    pid->max_out = max_out;
    pid->attitude_coeff = attitude_coeff;
}

int16 steerPIDCalculate(PID_t *pid, int16 error) {
    int32 p_term, d_term, att_term;
    int16 pid_output;
    p_term = (int32) pid->Kp * error;
    d_term = (int32) pid->Kd * (error - pid->last_error);
    att_term = (int32) pid->attitude_coeff * imu660rb_gyro_z;
    pid->last_error = error;
    pid_output = (p_term + d_term + att_term) / 100;
    return func_limit(pid_output, pid->max_out);
}

int16 motorPIDCalculate(PID_t *pid, int16 target, int16 actual) {
    int32 err, delta_err, delta2_err, delta_u;
//    if (actual - target > 100) { //BangBang~
//        gpio_set_level((gpio_pin_enum) pid->which ? motorDirR : motorDirL, 0);
//        return c2;
//    }
    err = target - actual;
    delta_err = err - pid->last_error;
    delta2_err = delta_err - (pid->last_error - pid->llast_error);
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * delta2_err;
    pid->llast_error = pid->last_error;
    pid->last_error = err;
    pid->base += delta_u / 100;
    pid->base = func_limit(pid->base, pid->max_out);
    gpio_set_level((gpio_pin_enum) pid->which ? motorDirR : motorDirL, pid->base >= 0);
    return func_abs(pid->base);
}

static void pitHandler() {
    encoderR = -encoder_get_count(PWMA_ENCODER) << 1;
    encoderL = encoder_get_count(PWMC_ENCODER) << 1;
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
    pwm_set_duty(PWME_CH4P_PA6, 3060 + steerPIDCalculate(&steerPid, duty));
    if (smoothStop || mustStop) {
        pwm_set_duty(motorPwmR, 0);
        pwm_set_duty(motorPwmL, 0);
        pwm_set_duty(pressPwmL, 0);
        pwm_set_duty(pressPwmR, 0);
    } else {
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, setSpeed + (!R ? 0 : setSpeed * 8 / R), encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, setSpeed - (!R ? 0 : setSpeed * 8 / R), encoderL));
        pwm_set_duty(pressPwmL, 900);
        pwm_set_duty(pressPwmR, 900);
    }
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy;
    int8 d;
    if (count <= 5) return;
    pos = func_limit_ab(aimPos, 5, count - 1);
    dx = MT9V03X_H - (roadLine[pos].x + roadLine[pos - 1].x + roadLine[pos - 2].x) / 3 + DEVICE_H;
    dy = 91 - (roadLine[pos].y + roadLine[pos - 1].y + roadLine[pos - 2].y) / 3;
    d = atan2_int(2 * DEVICE_H * dy, dx * dx + dy * dy);
    if (!display) ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
//    pos = func_limit_ab(c2, 5, count - 1);
//    dx = MT9V03X_H - roadLine[pos].x;
//    dy = 94 - roadLine[pos].y;
//    K = !dy ? 0 : 20 * dx / dy;
//    if (!display) ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
    //ips200_show_int16(0, 20, K);
    // if (RUN_MODE != RAMP && c5) {
    //     dy = 94 - roadLine[c5].y;
    //     //ips200_show_int16(0, 40, dy);
    //     if ((d <= 0 && dy <= 0) || (d >= 0 && dy >= 0)) d += dy;
    // }
    setServoDuty(func_limit(d, 30));
}

void setServoDuty(int8 d) {
    int16 t = tani[func_limit_ab(func_abs((int16) d * 170 / 100), 0, 30)];
    if (!display) ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 20, d);
    R = !t ? 0 : 20000 / t;
    //if (func_abs(K) > func_abs(R)) K = R;
    if (!handle)
        setSpeed = !R ? c4 : func_limit(sqrt_int(func_abs(R) * (uint32) c3), c4);
    if (d < 0) R = -R;
    // ips200_show_int16(0, 0, encoderL);
    // ips200_show_int16(0, 30, encoderR);
    //ips200_show_int16(0, 0, R);
    duty = (int16) d * 11;
}

int8 getDynamicAimPos() {
    int16 speed = encoderR + encoderL >> 1;
    return speed / 10 + c5;
}

void motorInit(int16 p, int16 i, int16 d, int16 amm, int16 spd, int16 dh) {
    c1 = p;
    c2 = i;
    c3 = d;
    c4 = amm;
    c5 = spd;
    c6 = dh;
    PIDInit(&steerPid, 3,
             170, 0, 200, 2,
             333);
    PIDInit(&motorLPID, 0,
            2500, 55, 1000, 0,
            9500);
    PIDInit(&motorRPID, 1,
            2500, 55, 1000, 0,
            9500);
    pwm_init(PWME_CH4P_PA6, servoFreq, 3060);
    gpio_init(motorDirR, GPO, 1, GPO_PUSH_PULL);
    gpio_init(motorDirL, GPO, 1, GPO_PUSH_PULL);
    pwm_init(motorPwmR, 17000, 0);
    pwm_init(motorPwmL, 17000, 0);
    pwm_init(pressPwmR, 50, 900);
    pwm_init(pressPwmL, 50, 900);
    // 在50Hz的控制频率下，无刷电调转速 0%   为 500
    // 在50Hz的控制频率下，无刷电调转速 20%  为 600
    // 在50Hz的控制频率下，无刷电调转速 40%  为 700
    // 在50Hz的控制频率下，无刷电调转速 60%  为 800
    // 在50Hz的控制频率下，无刷电调转速 80%  为 900
    // 在50Hz的控制频率下，无刷电调转速 100% 为 1000
    encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
    encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    pit_ms_init(TIM1_PIT, 5, pitHandler);
}

const int16 tani[31] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577};