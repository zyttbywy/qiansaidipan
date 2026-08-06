#include "zf_common_headfile.h"
#include "siyuanshu.h"
#include "math.h"

/* ------------------------------------------------------------------ */
/*  全局状态                                                            */
/* ------------------------------------------------------------------ */

// PI 控制器积分项
static float I_ex = 0.0f, I_ey = 0.0f, I_ez = 0.0f;

// 单位四元数（初始态：水平静止）
float Q_info_q0 = 1.0f, Q_info_q1 = 0.0f,
      Q_info_q2 = 0.0f, Q_info_q3 = 0.0f;

// 欧拉角输出：yaw单位rad，roll/pitch单位deg
float eulerAngle_pitch = 0.0f;
float eulerAngle_roll  = 0.0f;
float eulerAngle_yaw   = 0.0f;

// Mahony 互补滤波参数 ———— 可调
// kp: 加速度计修正强度，越大收敛越快但对振动越敏感
// ki: 陀螺仪零偏积分补偿，消除长期漂移，通常远小于 kp
static const float icm_kp = 0.8f;
static const float icm_ki = 0.005f;
static const float GYRO_DEADBAND = 0.02618f; // 1.5 deg/s in rad/s

// 陀螺仪零偏（由 gyroOffsetInit 计算）
float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f;

// 滤波后的传感器数据（单位：g / rad/s）
static float Angle_ax = 0.0f, Angle_ay = 0.0f, Angle_az = 0.0f;
static float Angle_gx = 0.0f, Angle_gy = 0.0f, Angle_gz = 0.0f;

/* ------------------------------------------------------------------ */
/*  陀螺仪零偏标定                                                      */
/*  上电静置后调用一次，设备须保持静止                                   */
/* ------------------------------------------------------------------ */
void gyroOffsetInit(void)
{
    gx_offset = 0.0f;
    gy_offset = 0.0f;
    gz_offset = 0.0f;

    // 等待传感器稳定（500 ms）
    system_delay_ms(1000);

    const int   SAMPLES = 500;
    const float INV_N   = 1.0f / SAMPLES;

    for (int i = 0; i < SAMPLES; ++i)
    {
        imu660ra_get_gyro();
        gx_offset += (float)imu660ra_gyro_x;
        gy_offset += (float)imu660ra_gyro_y;
        gz_offset += (float)imu660ra_gyro_z;
        system_delay_ms(3);
    }

    gx_offset *= INV_N;
    gy_offset *= INV_N;
    gz_offset *= INV_N;
}

void gyroOffsetTrackStill(void)
{
    const float BIAS_ALPHA = 0.02f;

    gx_offset += ((float)imu660ra_gyro_x - gx_offset) * BIAS_ALPHA;
    gy_offset += ((float)imu660ra_gyro_y - gy_offset) * BIAS_ALPHA;
    gz_offset += ((float)imu660ra_gyro_z - gz_offset) * BIAS_ALPHA;
}

/* ------------------------------------------------------------------ */
/*  传感器数据读取与单位换算                                             */
/*  每个控制周期（delta_T）开始时调用                                    */
/* ------------------------------------------------------------------ */
void icmGetValues(void)
{
    // ---- 加速度计 ----
    // 转换系数：使用库的 transition_factor，自动适配量程配置
    // 一阶低通滤波：alpha 越小越平滑但延迟越大，可调范围 0.2 ~ 0.8
    const float ALPHA     = 0.5f;
    const float ONE_ALPHA = 1.0f - ALPHA;
    const float ACC_SCALE = 1.0f / imu660ra_transition_factor[0];

    imu660ra_get_acc();

    Angle_ax = ALPHA * ((float)imu660ra_acc_x * ACC_SCALE) + ONE_ALPHA * Angle_ax;
    Angle_ay = ALPHA * ((float)imu660ra_acc_y * ACC_SCALE) + ONE_ALPHA * Angle_ay;
    Angle_az = ALPHA * ((float)imu660ra_acc_z * ACC_SCALE) + ONE_ALPHA * Angle_az;

    // ---- 陀螺仪 ----
    // 转换系数：LSB -> rad/s，使用库的 transition_factor 自动适配量程
    const float GYRO_SCALE = (1.0f / imu660ra_transition_factor[1]) * (3.14159265359f / 180.0f);

    imu660ra_get_gyro();

    float gx_raw = ((float)imu660ra_gyro_x - gx_offset) * GYRO_SCALE;
    float gy_raw = ((float)imu660ra_gyro_y - gy_offset) * GYRO_SCALE;
    float gz_raw = ((float)imu660ra_gyro_z - gz_offset) * GYRO_SCALE;

    // 死区：低于阈值的角速度视为零，抑制静止漂移
    if (fabsf(gx_raw) < GYRO_DEADBAND) gx_raw = 0.0f;
    if (fabsf(gy_raw) < GYRO_DEADBAND) gy_raw = 0.0f;
    if (fabsf(gz_raw) < GYRO_DEADBAND) gz_raw = 0.0f;

    Angle_gx = gx_raw;
    Angle_gy = gy_raw;
    Angle_gz = gz_raw;
}

/* ------------------------------------------------------------------ */
/*  Mahony AHRS 姿态解算                                               */
/*  互补滤波 + PI 控制器补偿陀螺仪漂移                                   */
/*  参考：Mahony et al., IEEE Trans. Automat. Contr., 2008             */
/* ------------------------------------------------------------------ */
void icmAHRSupdate(void)
{
    // ---------- 0. 读取当前四元数 ----------
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;

    // ---------- 1. 加速度计数据归一化 ----------
    float ax = Angle_ax, ay = Angle_ay, az = Angle_az;
    float norm_sq = ax * ax + ay * ay + az * az;

    // 加速度异常时跳过 PI 修正，但陀螺仪积分正常进行
    int acc_valid = (norm_sq > 1e-4f && norm_sq < 4.0f);

    if (acc_valid)
    {
        float norm = 1.0f / sqrtf(norm_sq);
        ax *= norm;
        ay *= norm;
        az *= norm;
    }

    // ---------- 2. 由当前四元数估计重力方向（机体系） ----------
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;

    // ---------- 3. 误差（加速度计观测 × 姿态估计） ----------
    float ex, ey, ez;
    if (acc_valid)
    {
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = 0.0f;  // 6轴姿态没有磁力计，yaw不可观测，禁止yaw轴PI修正

        // ---------- 4. PI 控制器修正 ----------
        I_ex += icm_ki   * ex * delta_T;
        I_ey += icm_ki   * ey * delta_T;
        I_ez = 0.0f;
    }
    else
    {
        // 加速度异常：跳过 PI 修正，仅靠陀螺仪积分维持姿态
        ex = 0.0f;
        ey = 0.0f;
        ez = 0.0f;
    }

    // 积分抗饱和：限制积分项不超过 ±0.1 rad/s
    if (I_ex >  0.1f) I_ex =  0.1f;
    if (I_ex < -0.1f) I_ex = -0.1f;
    if (I_ey >  0.1f) I_ey =  0.1f;
    if (I_ey < -0.1f) I_ey = -0.1f;
    I_ez = 0.0f;

    // 将 PI 输出叠加到陀螺仪角速度
    float gx = Angle_gx + icm_kp * ex + I_ex;
    float gy = Angle_gy + icm_kp * ey + I_ey;
    float gz = Angle_gz;

    // ---------- 5. 四元数微分方程（一阶欧拉积分） ----------
    float halfT = HALF_T;
    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * halfT;
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * halfT;
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * halfT;
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * halfT;

    // ---------- 6. 四元数归一化 ----------
    norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    float norm = 1.0f / sqrtf(norm_sq);
    Q_info_q0 = q0 * norm;
    Q_info_q1 = q1 * norm;
    Q_info_q2 = q2 * norm;
    Q_info_q3 = q3 * norm;

    // ---------- 7. 四元数 -> 欧拉角（ZYX 顺序） ----------
    float _q0 = Q_info_q0, _q1 = Q_info_q1,
          _q2 = Q_info_q2, _q3 = Q_info_q3;

    float sinp = 2.0f * (_q0 * _q2 - _q1 * _q3);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    eulerAngle_pitch = asinf(sinp) * 57.29577951f;

    eulerAngle_roll = atan2f(2.0f * (_q2 * _q3 + _q0 * _q1),
                             1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2))
                      * 57.29577951f;

    eulerAngle_yaw  = atan2f(2.0f * (_q1 * _q2 + _q0 * _q3),
                             1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
}

/* ------------------------------------------------------------------ */
/*  显示函数（IPS200 屏幕）                                              */
/* ------------------------------------------------------------------ */
void imu660ra_euler_show(void)
{
    ips200_show_float(0,  0, eulerAngle_yaw,   3, 2);
    ips200_show_float(0, 16, eulerAngle_roll,  3, 2);
    ips200_show_float(0, 32, eulerAngle_pitch, 3, 2);
    ips200_show_int(80,  0, imu660ra_gyro_x, 5);
    ips200_show_int(80, 16, imu660ra_gyro_y, 5);
    ips200_show_int(80, 32, imu660ra_gyro_z, 5);
    ips200_show_int(80, 48, imu660ra_acc_x,  5);
    ips200_show_int(80, 64, imu660ra_acc_y,  5);
    ips200_show_int(80, 80, imu660ra_acc_z,  5);
}
