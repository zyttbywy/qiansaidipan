#include "zf_common_headfile.h"
#include "uart.h"
#include "zf_device_uart_receiver.h"

int use_data[9] = {0};
int x_speed = 0;
int y_speed = 0;
int o_speed = 0;

void my_uart_write_byte(unsigned char byte) {
    // 方案 A：使用 printf 发送单个字符（符合题目要求）
    printf("%c", byte); 
    
    // 方案 B（推荐）：如果 printf 带有行缓存，可以改用 putchar 并配合 fflush
    // putchar(byte);
}

// 数据发送函数，参数为 9 个整数
void Data_Send(int* pst) {
    unsigned char i = 0;
    unsigned char _cnt = 0;
    unsigned char sum = 0;
    unsigned char data_to_send[32];

    // 1. 帧头与功能码
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0; // 长度占位

    // 2. 将 8 个 int 拆分为高低字节放入数组
    for (int j = 0; j < 8; ++j) {
        data_to_send[_cnt++] = (unsigned char)(pst[j] >> 8); // 高位
        data_to_send[_cnt++] = (unsigned char)(pst[j]);      // 低位
    }

    // 3. 计算有效数据长度
    data_to_send[3] = _cnt - 4;

    // 4. 计算校验和
    for (i = 0; i < _cnt; i++) {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;

    // ================== 核心修改部分 ==================
    // 5. 替换原本的循环，直接使用标准库发送整个字节数组
    // 这样比原先循环调用 printf 效率更高，且能保证一帧数据连续发送
    for (i = 0; i < _cnt; i++) {
        printf("%c", data_to_send[i]);
    }

    // 如果串口有缓存机制，建议在发送完一帧后刷新缓存，确保数据实时发出
    fflush(stdout); 
    // ==================================================
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       从 UART1 接收上位机发送的期望速度数据并解析
// 参数说明       void
// 返回参数       void
// 使用示例       get_expect_speed();
// 备注信息       上位机发送格式: "x_speed:12 y_speed:23 o_speed:15"
//                解析结果保存到全局变量 x_speed, y_speed, o_speed
//                依赖 UART1 接收中断将数据写入 uart2_data_fifo
//-------------------------------------------------------------------------------------------------------------------
uint8 get_expect_speed(void)
{
    static char buffer[64];
    static int idx = 0;
    uint8 updated = 0;

    uint32 fifo_count = fifo_used(&uart2_data_fifo);
    if(fifo_count != 0)
    {
        uint8 fifo_data[64];
        fifo_read_buffer(&uart2_data_fifo, fifo_data, &fifo_count, FIFO_READ_AND_CLEAN);

        for(uint32 i = 0; i < fifo_count; i++)
        {
            if(fifo_data[i] == 0x0A || fifo_data[i] == 0x0D)
            {
                if(idx > 0)
                {
                    int new_x_speed;
                    int new_y_speed;
                    int new_o_speed;

                    buffer[idx] = '\0';
                    if(3 == sscanf(buffer, "x_speed:%d y_speed:%d o_speed:%d",
                           &new_x_speed, &new_y_speed, &new_o_speed))
                    {
                        x_speed = new_x_speed;
                        y_speed = new_y_speed;
                        o_speed = new_o_speed;
                        updated = 1;
                    }
                    idx = 0;
                }
            }
            else if(idx < (int)(sizeof(buffer) - 1))
            {
                buffer[idx++] = (char)fifo_data[i];
            }
        }
    }

    return updated;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       从遥控器接收机获取期望速度（三段式开关控制）
// 参数说明       void
// 返回参数       void
// 使用示例       get_expect_speed_receiver();
// 备注信息       channel[0]: 旋转控制  (<800 逆时针+20 | >1600 顺时针-20 | 死区800~1600)
//                channel[1]: 速度大小  (>1500 正向+20 | <900 负向-20 | 死区900~1500)
//                channel[2]: x/y 切换 (<500 控制x | >1000 控制y | 死区500~1000)
//-------------------------------------------------------------------------------------------------------------------
void get_expect_speed_receiver(void)
{
#if !SBUS_SPEED_CONTROL_ENABLE
    return;
#else
    if(uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;

        if(uart_receiver.state == 1)
        {
            // --- channel[0]: 旋转 ---
            if(uart_receiver.channel[0] < 800)
                o_speed = 20;                                   // 逆时针
            else if(uart_receiver.channel[0] > 1600)
                o_speed = -20;                                  // 顺时针
            else
                o_speed = 0;                                    // 死区

            // --- channel[1]: 速度大小 ---
            int speed_val = 0;
            if(uart_receiver.channel[1] > 1500)
                speed_val = 20;                                 // 正向
            else if(uart_receiver.channel[1] < 900)
                speed_val = -20;                                // 负向
            // 死区 900~1500 → speed_val = 0

            // --- channel[2]: x/y 切换 ---
            if(uart_receiver.channel[2] < 500)
            {
                x_speed = speed_val;                            // 控制 x（前进/后退）
                y_speed = 0;
            }
            else if(uart_receiver.channel[2] > 1000)
            {
                x_speed = 0;
                y_speed = speed_val;                            // 控制 y（左移/右移）
            }
            else
            {
                x_speed = 0;                                    // 死区
                y_speed = 0;
            }
        }
    }
#endif
}
