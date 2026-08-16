#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

#define LEFT_UP_WHELL_GPIO P13_3
#define RIGHT_UP_WHELL_GPIO P20_8
#define LEFT_DOWN_WHELL_GPIO P00_3
#define RIGHT_DOWN_WHELL_GPIO P33_5

#define LEFT_UP_WHELL_PWM ATOM3_CH7_P13_2
#define RIGHT_UP_WHELL_PWM ATOM2_CH7_P20_7
#define LEFT_DOWN_WHELL_PWM ATOM0_CH1_P00_2
#define RIGHT_DOWN_WHELL_PWM ATOM3_CH0_P33_4

#define LEFT_UP_FORWARD 0
#define LEFT_UP_BACKWARD 1
#define RIGHT_UP_FORWARD 0
#define RIGHT_UP_BACKWARD 1
#define LEFT_DOWN_FORWARD 1
#define LEFT_DOWN_BACKWARD 0
#define RIGHT_DOWN_FORWARD 1
#define RIGHT_DOWN_BACKWARD 0

// 1: zero wheel targets disable PWM so the chassis can be pushed by hand.
// 0: keep the original active zero-speed PID holding behavior.
#define ZERO_SPEED_COAST_ENABLE 1

extern int left_up_goal;
extern int right_up_goal;
extern int left_down_goal;
extern int right_down_goal;

extern int left_up_speed;
extern int right_up_speed;
extern int left_down_speed;
extern int right_down_speed;

extern int pwm_left_up;
extern int pwm_right_up;
extern int pwm_left_down;
extern int pwm_right_down;

extern int lu[3];
extern int ru[3];
extern int ld[3];
extern int rd[3];

extern double x;
extern double y;
extern double o;
extern double odom_vx;
extern double odom_vy;
extern double odom_wz;

void speed_compute(int x_speed, int y_speed, int o_speed);
void speed_control(void);
void odometry_update(uint8 use_imu_yaw);

#endif // _CONTROL_H_
