/*********************************************************************************************************************
* ygqt_ips130.h
* 
* 文件名称          ygqt_ips130.h
* 队伍名称          厦门理工_阳光青提
* 功能说明          1.30寸 IPS 液晶屏驱动头文件 (基于逐飞 zf_device_ips114 修改适配)
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 
* 本文件是 基于 SEEKFREE 逐飞科技 TC264 Opensourec Library 即（TC264 开源库）完成的
*
* 修改记录
* 日期              作者                备注
* 2026-05-10       xiaogaga            基于逐飞 ips114 库创建，适配 1.30寸 ST7789 (240x240) 屏幕
********************************************************************************************************************/
/********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚              单片机管脚
*                  SCL                 查看 ygqt_ips130.h 中 IPS130_SCL_PIN 宏定义
*                  SDA                 查看 ygqt_ips130.h 中 IPS130_SDA_PIN 宏定义
*                  RST                 查看 ygqt_ips130.h 中 IPS130_RST_PIN 宏定义
*                  DC                  查看 ygqt_ips130.h 中 IPS130_DC_PIN 宏定义
*                  CS                  查看 ygqt_ips130.h 中 IPS130_CS_PIN 宏定义
*                  BLK                 查看 ygqt_ips130.h 中 IPS130_BLK_PIN 宏定义
*                  VCC                 3.3V电源
*                  GND                 电源地
*                  最大分辨率 240 * 240
*                  ------------------------------------
********************************************************************************************************************/

#ifndef _YGQT_IPS130_H_
#define _YGQT_IPS130_H_

#include "zf_common_typedef.h"

//=================================================定义 IPS130 基本配置================================================
#define IPS130_USE_SOFT_SPI             (0)                                     // 默认使用硬件 SPI 方式驱动 建议使用硬件 SPI 方式驱动
#if IPS130_USE_SOFT_SPI                                                         // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 SPI 驱动==================================================
#define IPS130_SOFT_SPI_DELAY           (0 )                                    // 软件 SPI 的时钟延时周期 数值越小 SPI 通信速率越快
#define IPS130_SCL_PIN                  (P15_3)                                 // 软件 SPI SCK 引脚
#define IPS130_SDA_PIN                  (P15_5)                                 // 软件 SPI MOSI 引脚
//====================================================软件 SPI 驱动==================================================
#else
//====================================================硬件 SPI 驱动==================================================
#define IPS130_SPI_SPEED                (60*1000*1000)                          // 硬件 SPI 速率
#define IPS130_SPI                      (SPI_1)                                 // 硬件 SPI 号
#define IPS130_SCL_PIN                  (SPI1_SCLK_P10_2)                       // 硬件 SPI SCK 引脚
#define IPS130_SDA_PIN                  (SPI1_MOSI_P11_9)                       // 硬件 SPI MOSI 引脚
#define IPS130_SDA_IN_PIN               (SPI1_MISO_P11_3)                       // 定义SPI_MISO引脚  IPS没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
//====================================================硬件 SPI 驱动==================================================
#endif

#define IPS130_RST_PIN                  (P02_0)                                  // 液晶复位引脚定义
#define IPS130_DC_PIN                   (P02_2)                                 // 液晶命令位引脚定义
#define IPS130_CS_PIN                   (P02_1)                                  // CS 片选引脚
#define IPS130_BLK_PIN                  (P02_3)                                  // 液晶背光引脚定义

#define IPS130_DEFAULT_DISPLAY_DIR      ( IPS130_CROSSWISE_180        )          // 默认的显示方向
#define IPS130_DEFAULT_PENCOLOR         (RGB565_RED)                             // 默认的画笔颜色
#define IPS130_DEFAULT_BGCOLOR          (RGB565_WHITE)                           // 默认的背景颜色
#define IPS130_DEFAULT_DISPLAY_FONT     (IPS130_8X16_FONT)                       // 默认的字体模式

#define IPS130_DC(x)                    ((x) ? (gpio_high(IPS130_DC_PIN))  : (gpio_low(IPS130_DC_PIN)))
#define IPS130_RST(x)                   ((x) ? (gpio_high(IPS130_RST_PIN)) : (gpio_low(IPS130_RST_PIN)))
#define IPS130_CS(x)                    ((x) ? (gpio_high(IPS130_CS_PIN))  : (gpio_low(IPS130_CS_PIN)))
#define IPS130_BLK(x)                   ((x) ? (gpio_high(IPS130_BLK_PIN)) : (gpio_low(IPS130_BLK_PIN)))
//=================================================定义 IPS130 基本配置================================================


//=================================================定义 IPS130 参数结构体===============================================
typedef enum
{
    IPS130_PORTAIT                      = 0,                                     // 竖屏模式
    IPS130_PORTAIT_180                  = 1,                                     // 竖屏模式  旋转180
    IPS130_CROSSWISE                    = 2,                                     // 横屏模式
    IPS130_CROSSWISE_180                = 3,                                     // 横屏模式  旋转180
}ips130_dir_enum;

typedef enum
{
    IPS130_6X8_FONT                     = 0,                                     // 6x8      字体
    IPS130_8X16_FONT                    = 1,                                     // 8x16     字体
    IPS130_16X16_FONT                   = 2,                                     // 16x16    字体 目前不支持
}ips130_font_size_enum;

extern  uint16  ips130_width_max ;
extern  uint16  ips130_height_max;

//=================================================定义 IPS130 参数结构体===============================================

//=================================================声明 IPS130 基础函数================================================
void    ips130_clear                    (void);                                                                             // IPS130 清屏函数
void    ips130_full                     (const uint16 color);                                                               // IPS130 屏幕填充函数
void    ips130_set_dir                  (ips130_dir_enum dir);                                                              // IPS130 设置显示方向
void    ips130_set_font                 (ips130_font_size_enum font);                                                       // IPS130 设置显示字体
void    ips130_set_color                (const uint16 pen, const  uint16 bgcolor);                                          // IPS130 设置显示颜色
void    ips130_draw_point               (uint16 x, uint16 y, const uint16 color);                                           // IPS130 画点函数
void    ips130_draw_line                (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);   // IPS130 画线函数

void    ips130_show_char                (uint16 x, uint16 y, const char dat);                                               // IPS130 显示字符
void    ips130_show_string              (uint16 x, uint16 y, const char dat[]);                                             // IPS130 显示字符串
void    ips130_show_int                 (uint16 x,uint16 y, const int32 dat, uint8 num);                                    // IPS130 显示32位有符号 (去除整数部分无效的0)
void    ips130_show_uint                (uint16 x,uint16 y, const uint32 dat, uint8 num);                                   // IPS130 显示32位无符号 (去除整数部分无效的0)
void    ips130_show_float               (uint16 x,uint16 y, const double dat, uint8 num, uint8 pointnum);

void    ips130_show_binary_image        (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);                     // IPS130 显示二值图像 数据每八个点组成一个字节数据
void    ips130_show_gray_image          (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold);    // IPS130 显示 8bit 灰度图像 带二值化阈值
void    ips130_show_rgb565_image        (uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 color_mode);  // IPS130 显示 RGB565 彩色图像

void    ips130_show_wave                (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);               // IPS130 显示波形
void    ips130_show_chinese             (uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number, const uint16 color);                                // IPS130 汉字显示
void    ips130_init                     (void);                                                                             // 1.30寸 IPS液晶初始化
void    ips130_show_image               (uint8 image_index);                                                               // IPS130 显示预置图片 (0-第一张 1-第二张)                                                                             // 1.30寸 IPS液晶初始化
//=================================================声明 IPS130 基础函数================================================


//=================================================声明 IPS130 扩展函数================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS130 显示小钻风图像
// 参数说明     p               图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     void
// 使用示例     ips130_displayimage7725(ov7725_image_binary[0], OV7725_W, OV7725_H);
// 备注信息     拓展的一键显示函数，默认无缩放，从屏幕坐标起始点开始显示
//-------------------------------------------------------------------------------------------------------------------
#define ips130_displayimage7725(p, width, height)       (ips130_show_binary_image(0, 0, (p), OV7725_W, OV7725_H, (width), (height)))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS130 显示总钻风图像
// 参数说明     p               图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     void
// 使用示例     ips130_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
// 备注信息     拓展的一键显示函数，默认无缩放，从屏幕坐标起始点开始显示
//-------------------------------------------------------------------------------------------------------------------
#define ips130_displayimage03x(p, width, height)        (ips130_show_gray_image(0, 0, (p), MT9V03X_W, MT9V03X_H, (width), (height), 0))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS130 显示凌瞳图像
// 参数说明     p               图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     void
// 使用示例     ips130_displayimage8660(scc8660_image[0], SCC8660_W, SCC8660_H);
// 备注信息     拓展的一键显示函数，默认无缩放，从屏幕坐标起始点开始显示
//-------------------------------------------------------------------------------------------------------------------
#define ips130_displayimage8660(p, width, height)       (ips130_show_rgb565_image(0, 0, (p), SCC8660_W, SCC8660_H, (width), (height), 1))
//=================================================声明 IPS130 扩展函数================================================

#endif
