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

// Gimbal servo test outputs. ATOM1 is used to avoid the existing ATOM0_CH1/P00_2 wheel output.
#define GIMBAL_SERVO_1_PWM ATOM1_CH0_P00_0
#define GIMBAL_SERVO_2_PWM ATOM1_CH1_P00_1
#define GIMBAL_SERVO_MIN_ANGLE 0
#define GIMBAL_SERVO_MAX_ANGLE 150
#define GIMBAL_SERVO_CENTER_ANGLE 75
#define GIMBAL_SERVO_MIN_DUTY 333
#define GIMBAL_SERVO_MAX_DUTY 1167
#define GIMBAL_SERVO_CENTER_DUTY 750

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
extern int gimbal_servo1_angle;
extern int gimbal_servo2_angle;

void speed_compute(int x_speed, int y_speed, int o_speed);
void speed_control(void);
void odometry_update(uint8 use_imu_yaw);
void gimbal_servo_init(void);
void gimbal_servo_set_angle(uint8 servo_index, int angle);
void gimbal_servo_test(void);

#endif // _CONTROL_H_
