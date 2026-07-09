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

volatile int16 encoderL, encoderR, setSpeed = 0, handle = 0, duty = 0;
const int16 tani[31];
extern int16 RUN_MODE;
extern uint8 switch1, borderIn, mustStop;
static uint8 DEVICE_H = 30, send_buff[32];
int16 R, c1, c2, c3, c4, c5, c6, c7, LSetSpeed, RSetSpeed, ct = 0, ct2 = 0;
int16 currentDuty, longc = 300, LPWM, RPWM;
int8 smoothStop, FD;

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
    int32 err, delta_err, delta_u;
    err = target - actual;
    delta_err = err - pid->last_error;
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * delta_err - (pid->last_error - pid->llast_error);
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
    pwm_set_duty(PWME_CH4P_PA6, 2835 + steerPIDCalculate(&steerPid, duty));
    if (smoothStop || mustStop) {
        pwm_set_duty(motorPwmR, 0);
        pwm_set_duty(motorPwmL, 0);
        pwm_set_duty(pressPwmL, 0);
        pwm_set_duty(pressPwmR, 0);
    } else {
        RSetSpeed = setSpeed + (!R ? 0 : setSpeed * 8 / R);
        LSetSpeed = setSpeed - (!R ? 0 : setSpeed * 8 / R);
        // if (longc > 150) RSetSpeed = LSetSpeed = c4;
        // else RSetSpeed = LSetSpeed = c5;
        RPWM = motorPIDCalculate(&motorRPID, RSetSpeed, encoderR);
        LPWM = motorPIDCalculate(&motorLPID, LSetSpeed, encoderL);
        pwm_set_duty(motorPwmR, RPWM);
        pwm_set_duty(motorPwmL, LPWM);
        if (!handle) {
            pwm_set_duty(pressPwmL, c7);
            pwm_set_duty(pressPwmR, c7);
        }
    }
    // longc--;
    // if (!longc) longc = 300;
}

void up() {
    sprintf(send_buff, "speed: %d, %d, %d\n", LSetSpeed, encoderL, LPWM);
    if (!mustStop) wireless_uart_send_buffer(send_buff, 32);
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy;
    int8 d;
    if (count <= 5) return;
    pos = func_limit_ab(aimPos, 5, count - 1);
    dx = MT9V03X_H - (roadLine[pos].x + roadLine[pos - 1].x + roadLine[pos - 2].x) / 3 + DEVICE_H;
    dy = 91 - (roadLine[pos].y + roadLine[pos - 1].y + roadLine[pos - 2].y) / 3;
    d = atan2_int(2 * DEVICE_H * dy, dx * dx + dy * dy);
#if (DEBUG_MODE)
    if (switch1) ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
#endif

    pos = func_limit_ab(aimPos + 5, 5, count - 1);
    dx = roadLine[0].x - roadLine[pos].x + DEVICE_H;
    dy = 91 - roadLine[pos].y;
    FD = atan2_int(2 * DEVICE_H * dy, dx * dx + dy * dy);

    setServoDuty(func_limit(d, 30));
}

void setServoDuty(int8 d) {
    int16 data t, ts;
    //d = c1;
    currentDuty = (int16) d * 170 / 100;
    t = tani[func_limit_ab(func_max(func_abs(currentDuty), func_abs(FD * 17 / 10)), 0, 30)];
#if (DEBUG_MODE)
    if (switch1) ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 20, currentDuty);
#endif
    R = !t ? 0 : 20000 / t;
    if (borderIn) ts = func_min(c5, 320);
    else ts = c5;
    if (!handle)
        setSpeed = !R ? ts : func_limit(sqrt_int(func_abs(R) * (uint32) c4), ts);
    else setSpeed = func_min(setSpeed, !R ? c5 : func_limit(sqrt_int(func_abs(R) * (uint32) c4), c5));
    if (d < 0) R = -R;
    duty = (int16) d * 11;
}

int8 getDynamicAimPos() {
    int16 speed = encoderR + encoderL >> 1;
    return speed / 10 + c6;
}

void motorInit(int16 c11, int16 c22, int16 c33, int16 c44, int16 c55, int16 c66, int16 c77) {
    c1 = c11;
    c2 = c22;
    c3 = c33;
    c4 = c44;
    c5 = c55;
    c6 = c66;
    c7 = c77;
    PIDInit(&steerPid, 3,
             c1, 0, c2, c3,
             375);
    PIDInit(&motorLPID, 0,
            3000, 100, 50, 0,
            9000);
    PIDInit(&motorRPID, 1,
            3000, 100, 50, 0,
            9000);
    pwm_init(PWME_CH4P_PA6, servoFreq, 2835);
    gpio_init(motorDirR, GPO, 1, GPO_PUSH_PULL);
    gpio_init(motorDirL, GPO, 1, GPO_PUSH_PULL);
    pwm_init(motorPwmR, 17000, 0);
    pwm_init(motorPwmL, 17000, 0);
    pwm_init(pressPwmR, 50, 0);
    pwm_init(pressPwmL, 50, 0);
    // 在50Hz的控制频率下，无刷电调转速 0%   为 500
    // 在50Hz的控制频率下，无刷电调转速 20%  为 600
    // 在50Hz的控制频率下，无刷电调转速 40%  为 700
    // 在50Hz的控制频率下，无刷电调转速 60%  为 800
    // 在50Hz的控制频率下，无刷电调转速 80%  为 900
    // 在50Hz的控制频率下，无刷电调转速 100% 为 1000
    encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
    encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    pit_ms_init(TIM1_PIT, 5, pitHandler);
    //pit_ms_init(TIM3_PIT, 5, up);
}

const int16 tani[31] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577};