#include "zf_common_headfile.h"
#include "control.h"

int left_up_goal = 0;
int right_up_goal = 0;
int left_down_goal = 0;
int right_down_goal = 0;

int left_up_speed = 0;
int right_up_speed = 0;
int left_down_speed = 0;
int right_down_speed = 0;

int pwm_left_up = 0;
int pwm_right_up = 0;
int pwm_left_down = 0;
int pwm_right_down = 0;

int lu[3] = {0};
int ru[3] = {0};
int ld[3] = {0};
int rd[3] = {0};

float wp = 20; // 比例增益
float wi = 60; // 积分增益
float wd = 0.0; // 微分增益

double x = 0;
double y = 0;
double o = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       X型麦轮运动学逆解：将期望速度分解为四轮目标速度
// 参数说明       x_speed         前进方向期望速度（正=前进）
// 参数说明       y_speed         横向期望速度（正=右移）
// 参数说明       o_speed         旋转期望速度（正=逆时针）
// 返回参数       void
// 使用示例       speed_compute(x_speed, y_speed, o_speed);
// 备注信息       辊子从上方看呈X形：FL(/) FR(\) RL(\) RR(/)
//                结果保存到 left_up_goal / right_up_goal / left_down_goal / right_down_goal
//-------------------------------------------------------------------------------------------------------------------
void speed_compute(int x_speed, int y_speed, int o_speed){
    left_up_goal   =  x_speed + y_speed - o_speed;   // 左前轮 FL(/)
    right_up_goal  =  x_speed - y_speed + o_speed;   // 右前轮 FR(\)
    left_down_goal =  x_speed - y_speed - o_speed;   // 左后轮 RL(\)
    right_down_goal =  x_speed + y_speed + o_speed;  // 右后轮 RR(/)
}

void speed_control()
{

    lu[0] = left_up_goal - left_up_speed; // e(k)

    // 计算比例项 P = Kp * (e(k) - e(k-1))
    int luP = (int)(wp * (lu[0] - lu[1]));

    // 计算积分项 I = Ki * e(k)
    int luI = (int)(wi * lu[0]);

    // 计算微分项 D = Kd * (e(k) - 2 * e(k-1) + e(k-2))
    int luD = (int)(wd * (lu[0] - 2 * lu[1] + lu[2]));

    // 计算控制量 u(k) = P + I + D
    pwm_left_up += (int)((luP + luI + luD));

    // 更新误差历史
    lu[2] = lu[1]; // 更新 e(k-2)
    lu[1] = lu[0]; // 更新 e(k-1)

    // 右侧控制代码
    ru[0] = right_up_goal - right_up_speed; // e(k)

    // 计算比例项 P = Kp * (e(k) - e(k-1))
    int ruP = (int)(wp * (ru[0] - ru[1]));

    // 计算积分项 I = Ki * e(k)
    int ruI = (int)(wi * ru[0]);

    // 计算微分项 D = Kd * (e(k) - 2 * e(k-1) + e(k-2))
    int ruD = (int)(wd * (ru[0] - 2 * ru[1] + ru[2]));

    // 计算控制量 u(k) = P + I + D
    pwm_right_up += (int)((ruP + ruI + ruD));

    // 更新误差历史
    ru[2] = ru[1]; // 更新 e(k-2)
    ru[1] = ru[0]; // 更新 e(k-1)

    ld[0] = left_down_goal - left_down_speed; // e(k)

    // 计算比例项 P = Kp * (e(k) - e(k-1))
    int ldP = (int)(wp * (ld[0] - ld[1]));

    // 计算积分项 I = Ki * e(k)
    int ldI = (int)(wi * ld[0]);

    // 计算微分项 D = Kd * (e(k) - 2 * e(k-1) + e(k-2))
    int ldD = (int)(wd * (ld[0] - 2 * ld[1] + ld[2]));

    // 计算控制量 u(k) = P + I + D
    pwm_left_down += (int)((ldP + ldI + ldD));

    // 更新误差历史
    ld[2] = ld[1]; // 更新 e(k-2)
    ld[1] = ld[0]; // 更新 e(k-1)

    rd[0] = right_down_goal - right_down_speed; // e(k)

    // 计算比例项 P = Kp * (e(k) - e(k-1))
    int rdP = (int)(wp * (rd[0] - rd[1]));

    // 计算积分项 I = Ki * e(k)
    int rdI = (int)(wi * rd[0]);

    // 计算微分项 D = Kd * (e(k) - 2 * e(k-1) + e(k-2))
    int rdD = (int)(wd * (rd[0] - 2 * rd[1] + rd[2]));

    // 计算控制量 u(k) = P + I + D
    pwm_right_down += (int)((rdP + rdI + rdD));

    // 更新误差历史
    rd[2] = rd[1]; // 更新 e(k-2)
    rd[1] = rd[0]; // 更新 e(k-1)

    if (pwm_left_up > 9990)
    {

        pwm_left_up = 9990;
    }
    if (pwm_left_up < -9990)
    {

        pwm_left_up = -9990;
    }
    if (pwm_right_up > 9990)
    {

        pwm_right_up = 9990;
    }
    if (pwm_right_up < -9990)
    {

        pwm_right_up = -9990;
    }
    if (pwm_left_down > 9990)
    {

        pwm_left_down = 9990;
    }
    if (pwm_left_down < -9990)
    {
        
        pwm_left_down = -9990;
    }
    if (pwm_right_down > 9990)
    {

        pwm_right_down = 9990;
    }
    if (pwm_right_down < -9990)
    {

        pwm_right_down = -9990;
    }

    if (pwm_left_up < 0)
    {

        gpio_set_level(LEFT_UP_WHELL_GPIO, LEFT_UP_BACKWARD);
        pwm_set_duty(LEFT_UP_WHELL_PWM, -pwm_left_up);
    }
    else
    {

        gpio_set_level(LEFT_UP_WHELL_GPIO, LEFT_UP_FORWARD);
        pwm_set_duty(LEFT_UP_WHELL_PWM, pwm_left_up);
    }
    if (pwm_right_up < 0)
    {

        gpio_set_level(RIGHT_UP_WHELL_GPIO, RIGHT_UP_BACKWARD);
        pwm_set_duty(RIGHT_UP_WHELL_PWM, -pwm_right_up);
    }
    else
    {

        gpio_set_level(RIGHT_UP_WHELL_GPIO, RIGHT_UP_FORWARD);
        pwm_set_duty(RIGHT_UP_WHELL_PWM, pwm_right_up);
    }
    if( pwm_left_down < 0)
    {

        gpio_set_level(LEFT_DOWN_WHELL_GPIO, LEFT_DOWN_BACKWARD);
        pwm_set_duty(LEFT_DOWN_WHELL_PWM, -pwm_left_down);
    }
    else
    {

        gpio_set_level(LEFT_DOWN_WHELL_GPIO, LEFT_DOWN_FORWARD);
        pwm_set_duty(LEFT_DOWN_WHELL_PWM, pwm_left_down);
    }
    if( pwm_right_down < 0)
    {
        gpio_set_level(RIGHT_DOWN_WHELL_GPIO, RIGHT_DOWN_BACKWARD);
        pwm_set_duty(RIGHT_DOWN_WHELL_PWM, -pwm_right_down);
    }
    else
    {

        gpio_set_level(RIGHT_DOWN_WHELL_GPIO, RIGHT_DOWN_FORWARD);
        pwm_set_duty(RIGHT_DOWN_WHELL_PWM, pwm_right_down);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       X型麦轮里程计：由四轮速度积分计算绝对位姿
// 参数说明       void
// 返回参数       void
// 使用示例       odometry_update();  // 在 PIT 中断中调用（50ms 周期）
// 备注信息       结果保存到 x(坐标), y(坐标), o(朝向弧度, 归一化到[-π,π])
//                逆时针旋转 o 减小（为负）
//-------------------------------------------------------------------------------------------------------------------
void odometry_update(void)
{
    // ---- X型麦轮正运动学：轮速 → 车体速度 ----
    double vx = ( left_up_speed + right_up_speed + left_down_speed + right_down_speed) / 4.0;
    double vy = ( left_up_speed - right_up_speed - left_down_speed + right_down_speed) / 4.0;
    double vw = (-left_up_speed + right_up_speed - left_down_speed + right_down_speed) / 4.0;

    // ---- 积分（PIT 周期 50ms）----
    const double dt = 0.05;

    o -= vw * dt;                                       // 逆时针为负

    // 角度归一化到 [-π, π]
    while(o >  3.14159265358979) o -= 6.28318530717958;
    while(o < -3.14159265358979) o += 6.28318530717958;

    x += (vx * cos(o) - vy * sin(o)) * dt * 2.2;
    y += (vx * sin(o) + vy * cos(o)) * dt * 1.97;
}

void wheel_init(){

    pwm_init(LEFT_UP_WHELL_PWM, 25000, 0);
    pwm_init(RIGHT_UP_WHELL_PWM, 25000, 0);
    pwm_init(LEFT_DOWN_WHELL_PWM, 25000, 0);
    pwm_init(RIGHT_DOWN_WHELL_PWM, 25000, 0);

    gpio_init(LEFT_UP_WHELL_GPIO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(RIGHT_UP_WHELL_GPIO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LEFT_DOWN_WHELL_GPIO, GPO, 0, GPO_PUSH_PULL);
    gpio_init(RIGHT_DOWN_WHELL_GPIO, GPO, 0, GPO_PUSH_PULL);
}