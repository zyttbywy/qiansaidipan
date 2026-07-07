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
#include "zf_device_uart_receiver.h"
#pragma section all "cpu0_dsram"
int expect_speed = 0;
#define UART2_BAUDRATE          (115200)
#define UART2_TX_PIN            (UART2_TX_P33_9)
#define UART2_RX_PIN            (UART2_RX_P33_8)

uint8 uart2_rx_buffer[64];                                                      // UART1 接收数据缓冲区
fifo_struct uart2_data_fifo;                                                    // UART1 fifo 结构体
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************


int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    //ips200_set_dir(IPS200_CROSSWISE_180);
    ips130_init();
    ips130_show_string(0, 0, "Hello World");
    keyboard_init();
    wheel_init();

    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    encoder_dir_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7);
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);

    pit_ms_init(CCU60_CH0, 50);
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

    while (TRUE)
    {   
        // get_expect_speed_receiver();                                        // 从遥控器接收机获取期望速度
        get_expect_speed();
        speed_compute(x_speed, y_speed, o_speed);                           // 根据期望速度计算各轮目标速度

        // 显示三个通道原始值 (0~2047)，每行 20 像素
        // Y=0:  x_speed  y_speed  o_speed（期望速度，调试用）
        ips130_show_int(0, 0,  x_speed, 6);
        ips130_show_int(0,20,  y_speed, 6);
        ips130_show_int(0,40, o_speed, 6);

        // Y=20: 里程计位姿
        ips130_show_float(0, 60, x, 8, 6);
        ips130_show_float(0,80, y, 8, 6);
        ips130_show_float(0,100, o, 8, 6);  // o 弧度→度

        system_delay_ms(100);                                                // 延时 100ms
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    left_up_speed = -encoder_get_count(TIM5_ENCODER);
    right_up_speed = -encoder_get_count(TIM6_ENCODER);
    left_down_speed = encoder_get_count(TIM3_ENCODER);
    right_down_speed = encoder_get_count(TIM2_ENCODER); 
    odometry_update(); 
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

#pragma section all restore
// **************************** 代码区域 ****************************
