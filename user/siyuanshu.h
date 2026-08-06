#ifndef _siyuanshu_h_
#define _siyuanshu_h_

// 采样周期 5ms，频率 200Hz，与 IMU_UPDATE_PERIOD_MS 保持一致
#define delta_T     0.005f
#define HALF_T      (0.5f * delta_T)

// 欧拉角：yaw单位rad，pitch/roll单位deg
extern float eulerAngle_yaw;
extern float eulerAngle_pitch;
extern float eulerAngle_roll;

// 陀螺仪零偏
extern float gx_offset, gy_offset, gz_offset;

void  gyroOffsetInit(void);
void  gyroOffsetTrackStill(void);
float invSqrt(float x);
void  icmGetValues(void);
void  icmAHRSupdate(void);
void  imu660ra_euler_show(void);

#endif /* _siyuanshu_h_ */
