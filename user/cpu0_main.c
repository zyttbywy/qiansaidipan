/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "control.h"
#include "keyboard.h"
#include "uart.h"
#include "siyuanshu.h"
#include "zf_device_uart_receiver.h"
#pragma section all "cpu0_dsram"
int expect_speed = 0;
#define UART2_BAUDRATE          (115200)
#define UART2_TX_PIN            (UART2_TX_P33_9)
#define UART2_RX_PIN            (UART2_RX_P33_8)
#define IMU_UPDATE_PERIOD_MS    (5)
#define CONTROL_PERIOD_MS       (50)
#define CONTROL_PERIOD_DIV      (CONTROL_PERIOD_MS / IMU_UPDATE_PERIOD_MS)
#define COMMAND_TIMEOUT_MS      (300)
#define ODOM_POSITION_TO_M      (0.01)
#define ODOM_VELOCITY_TO_MPS    (0.01)
#define IMU_DISPLAY_X           (132)
#define IMU_DISPLAY_VALUE_X     (156)

uint8 uart2_rx_buffer[64];                                                      // UART1 接收数据缓冲区
fifo_struct uart2_data_fifo;                                                    // UART1 fifo 结构体
volatile uint32 system_ms = 0;
volatile uint8 odom_send_flag = 0;
uint32 odom_seq = 0;
uint32 last_command_ms = 0;
uint8 imu660ra_state = 1;
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static void odom_float_to_string(char *buffer, double value)
{
    int32 integer_part;
    uint32 fractional_part;
    uint32 divisor;
    uint32 offset;

    if(value < 0)
    {
        *buffer++ = '-';
        value = -value;
    }

    integer_part = (int32)value;
    fractional_part = (uint32)((value - integer_part) * 1000000.0 + 0.5);
    if(fractional_part >= 1000000)
    {
        integer_part++;
        fractional_part -= 1000000;
    }

    offset = zf_sprintf((int8 *)buffer, "%d.", integer_part);
    buffer += offset;
    divisor = 100000;
    while(divisor > 0)
    {
        *buffer++ = (char)('0' + (fractional_part / divisor) % 10);
        divisor /= 10;
    }
    *buffer = '\0';
}

static void send_odom_frame(void)
{
    int8 odom_buffer[256];
    char x_buffer[24];
    char y_buffer[24];
    char yaw_buffer[24];
    char vx_buffer[24];
    char vy_buffer[24];
    char wz_buffer[24];
    uint32 length;
    uint32 interrupt_state;
    uint32 timestamp_ms;
    double x_snapshot;
    double y_snapshot;
    double yaw_snapshot;
    double vx_snapshot;
    double vy_snapshot;
    double wz_snapshot;

    interrupt_state = interrupt_global_disable();
    timestamp_ms = system_ms;
    x_snapshot = x;
    y_snapshot = y;
    yaw_snapshot = o;
    vx_snapshot = odom_vx;
    vy_snapshot = odom_vy;
    wz_snapshot = odom_wz;
    interrupt_global_enable(interrupt_state);

    odom_float_to_string(x_buffer, x_snapshot * ODOM_POSITION_TO_M);
    odom_float_to_string(y_buffer, y_snapshot * ODOM_POSITION_TO_M);
    odom_float_to_string(yaw_buffer, yaw_snapshot);
    odom_float_to_string(vx_buffer, vx_snapshot * ODOM_VELOCITY_TO_MPS);
    odom_float_to_string(vy_buffer, vy_snapshot * ODOM_VELOCITY_TO_MPS);
    odom_float_to_string(wz_buffer, wz_snapshot);

    length = zf_sprintf(odom_buffer,
            "odom seq:%u ts_ms:%u x:%s y:%s yaw:%s vx:%s vy:%s wz:%s\r\n",
            odom_seq++, timestamp_ms, x_buffer, y_buffer, yaw_buffer, vx_buffer, vy_buffer, wz_buffer);
    uart_write_buffer(UART_2, (const uint8 *)odom_buffer, length);
}


int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    //ips200_set_dir(IPS200_CROSSWISE_180);
    ips130_init();
    ips130_show_string(0, 0, "Hello World");
    imu660ra_state = imu660ra_init();
    if(0 == imu660ra_state)
    {
        ips130_show_string(0, 20, "GYRO CAL");
        gyroOffsetInit();
        ips130_show_string(0, 20, "GYRO OK ");
    }
    keyboard_init();
    wheel_init();

    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    encoder_dir_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7);
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);

    pit_ms_init(CCU60_CH0, IMU_UPDATE_PERIOD_MS);
    uart_receiver_init();                                                       // 初始化串口接收机 (UART1 SBUS)
    uart_init(UART_2, UART2_BAUDRATE, UART2_TX_PIN, UART2_RX_PIN);   // 初始化 UART2 用于接收上位机速度指令
    fifo_init(&uart2_data_fifo, FIFO_DATA_8BIT, uart2_rx_buffer, 64);          // 初始化 UART2 fifo
    uart_rx_interrupt(UART_2, 1);                                               // 开启 UART2 接收中断
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    
    gpio_set_level(LEFT_UP_WHELL_GPIO, LEFT_UP_FORWARD);
    gpio_set_level(RIGHT_UP_WHELL_GPIO, RIGHT_UP_FORWARD);
    gpio_set_level(LEFT_DOWN_WHELL_GPIO, LEFT_DOWN_FORWARD);
    gpio_set_level(RIGHT_DOWN_WHELL_GPIO, RIGHT_DOWN_FORWARD);

    x = 0;
    y = 0;
    o = 0;
    odom_vx = 0;
    odom_vy = 0;
    odom_wz = 0;
    encoder_clear_count(TIM2_ENCODER);
    encoder_clear_count(TIM3_ENCODER);
    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
    last_command_ms = system_ms;

    while (TRUE)
    {   
        get_expect_speed_receiver();                                        // 从遥控器接收机获取期望速度
        if(get_expect_speed())
        {
            last_command_ms = system_ms;
        }
        if((system_ms - last_command_ms) > COMMAND_TIMEOUT_MS)
        {
            x_speed = 0;
            y_speed = 0;
            o_speed = 0;
        }
        speed_compute(x_speed, y_speed, o_speed);                           // 根据期望速度计算各轮目标速度
        if(odom_send_flag)
        {
            odom_send_flag = 0;
            send_odom_frame();
        }
        // 显示三个通道原始值 (0~2047)，每行 20 像素
        // Y=0:  x_speed  y_speed  o_speed（期望速度，调试用）
        ips130_show_int(0, 0,  x_speed, 6);
        ips130_show_int(0,20,  y_speed, 6);
        ips130_show_int(0,40, o_speed, 6);

        // Y=20: 里程计位姿
        ips130_show_float(0, 60, x, 8, 6);
        ips130_show_float(0,80, y, 8, 6);
        ips130_show_float(0,100, o, 8, 6);  // o 弧度→度
        ips130_show_string(IMU_DISPLAY_X, 0, (0 == imu660ra_state) ? "IMU OK " : "IMU ERR");
        ips130_show_string(IMU_DISPLAY_X, 20, "GX");
        ips130_show_string(IMU_DISPLAY_X, 40, "GY");
        ips130_show_string(IMU_DISPLAY_X, 60, "GZ");
        if(0 == imu660ra_state)
        {
            ips130_show_float(IMU_DISPLAY_VALUE_X, 20, ((double)imu660ra_gyro_x - gx_offset) / imu660ra_transition_factor[1], 5, 1);
            ips130_show_float(IMU_DISPLAY_VALUE_X, 40, ((double)imu660ra_gyro_y - gy_offset) / imu660ra_transition_factor[1], 5, 1);
            ips130_show_float(IMU_DISPLAY_VALUE_X, 60, ((double)imu660ra_gyro_z - gz_offset) / imu660ra_transition_factor[1], 5, 1);
        }
        else
        {
            ips130_show_float(IMU_DISPLAY_VALUE_X, 20, 0, 5, 1);
            ips130_show_float(IMU_DISPLAY_VALUE_X, 40, 0, 5, 1);
            ips130_show_float(IMU_DISPLAY_VALUE_X, 60, 0, 5, 1);
        }

        system_delay_ms(5);
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint8 control_period_count = 0;
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    system_ms += IMU_UPDATE_PERIOD_MS;

    if(0 == imu660ra_state)
    {
        icmGetValues();
        icmAHRSupdate();
    }

    control_period_count++;
    if(control_period_count >= CONTROL_PERIOD_DIV)
    {
        control_period_count = 0;
        left_up_speed = -encoder_get_count(TIM5_ENCODER);
        right_up_speed = -encoder_get_count(TIM6_ENCODER);
        left_down_speed = encoder_get_count(TIM3_ENCODER);
        right_down_speed = encoder_get_count(TIM2_ENCODER);
        if((0 == imu660ra_state)
                && (0 == x_speed) && (0 == y_speed) && (0 == o_speed)
                && (left_up_speed > -2) && (left_up_speed < 2)
                && (right_up_speed > -2) && (right_up_speed < 2)
                && (left_down_speed > -2) && (left_down_speed < 2)
                && (right_down_speed > -2) && (right_down_speed < 2))
        {
            gyroOffsetTrackStill();
        }
        odometry_update(0 == imu660ra_state);
        odom_send_flag = 1;
        //use_data[0] = left_up_speed;
        //use_data[1] = right_up_speed;
        //use_data[2] = left_down_speed;
        //use_data[3] = right_down_speed;
        //use_data[4] = expect_speed;
        //Data_Send(use_data);
        encoder_clear_count(TIM2_ENCODER);
        encoder_clear_count(TIM3_ENCODER);
        encoder_clear_count(TIM5_ENCODER);
        encoder_clear_count(TIM6_ENCODER);
        speed_control();
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
