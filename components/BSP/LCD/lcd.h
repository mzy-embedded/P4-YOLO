/**
 ******************************************************************************
 * @file    lcd.h
 * @brief   1.3寸 LCD 显示驱动 (ST7789, 4线SPI, 240x240 RGB565)
 * @note    适配 ESP32-P4 (esp_lcd_panel_io), 屏型号 M130T-240240-RGB-7
 *          分辨率 240x240, 16bit RGB565, 无全屏帧缓冲, 绘制即时刷新
 ******************************************************************************
 */

#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* 引脚定义 (SPI-LCD 排针接线, 1.3寸 7脚屏)                                     */
/*============================================================================*/
#define LCD_CLK_GPIO_PIN                GPIO_NUM_16         /* SPI时钟 (SCL) */
#define LCD_MOSI_GPIO_PIN               GPIO_NUM_17         /* SPI数据 (SDA) */
#define LCD_RST_GPIO_PIN                GPIO_NUM_18         /* 复位 (RES) */
#define LCD_DC_GPIO_PIN                 GPIO_NUM_19         /* 数据/命令 (DC) */
#define LCD_BL_GPIO_PIN                 GPIO_NUM_20         /* 背光 (BLK),若不使用置-1 */
/* 注意: 1.3寸 7脚屏无 CS 引脚, 板内已拉低, 驱动不驱动片选 (cs_gpio_num = -1) */

/*============================================================================*/
/* 分辨率                                                                       */
/*============================================================================*/
#define LCD_PIXEL_WIDTH                 240                 /* 显示区宽度(像素) */
#define LCD_PIXEL_HEIGHT                240                 /* 显示区高度(像素) */

/*============================================================================*/
/* 常用颜色 (RGB565)                                                            */
/*============================================================================*/
#define LCD_COLOR_BLACK                 0x0000
#define LCD_COLOR_NAVY                  0x000F
#define LCD_COLOR_DARKGREEN             0x03E0
#define LCD_COLOR_DARKCYAN              0x03EF
#define LCD_COLOR_MAROON                0x7800
#define LCD_COLOR_PURPLE                0x780F
#define LCD_COLOR_OLIVE                 0x7BE0
#define LCD_COLOR_LIGHTGREY             0xC618
#define LCD_COLOR_DARKGREY              0x7BEF
#define LCD_COLOR_BLUE                  0x001F
#define LCD_COLOR_GREEN                 0x07E0
#define LCD_COLOR_CYAN                  0x07FF
#define LCD_COLOR_RED                   0xF800
#define LCD_COLOR_MAGENTA               0xF81F
#define LCD_COLOR_YELLOW                0xFFE0
#define LCD_COLOR_WHITE                 0xFFFF
#define LCD_COLOR_ORANGE                0xFD20
#define LCD_COLOR_GREENYELLOW           0xAFE5
#define LCD_COLOR_PINK                  0xF81F

/*============================================================================*/
/* 字体大小 (宽像素数, 兼容 OLED 参数: 6x8 英文 / 8x16 中西文混排)                 */
/*============================================================================*/
#define LCD_6X8         6       /* 宽6像素 高8像素 (仅英文) */
#define LCD_8X16        8       /* 宽8像素 高16像素 (含中英文) */

/*============================================================================*/
/* 函数原型                                                                    */
/*============================================================================*/

/* 初始化 (含 SPI 总线 + 背光 + 复位 + ST7789 初始化序列 + 清屏) */
void LCD_Init(void);

/* 屏幕旋转 (0=0° 1=90° 2=180° 3=270°, 含 GRAM 偏移自动切换) */
void LCD_SetRotation(uint8_t Index);

/* 上电默认旋转方向 (改这里即可换默认方向)
 * 本工程取 0: 与移植前的 LCD_MADCTL_VALUE=0x00 / LCD_COL_OFFSET=0 完全等价;
 * 改成其它值会让摄像头预览画面跟着转。 */
#define LCD_ROTATION_INIT   0

/* LCD 访问互斥 (多任务同时绘制时, 把各自完整的绘制序列用 Begin/End 包住)
 * LCD_Begin 带回超时: 返回 1=已取到锁可绘制, 0=锁忙(超时, 调用方应跳过本次绘制) */
int  LCD_Begin(void);
void LCD_End(void);

/* 清屏 / 填充矩形 */
void LCD_Clear(uint16_t Color);
void LCD_FillRect(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color);

/* 绘图 */
void LCD_DrawPoint(uint16_t X, uint16_t Y, uint16_t Color);
void LCD_DrawLine(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color);
void LCD_DrawRect(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color);
void LCD_DrawCircle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Color);
void LCD_FillCircle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Color);

/* 文本显示 (ForeColor 前景色, BgColor 背景色) */
void LCD_ShowChar(uint16_t X, uint16_t Y, char Char, uint8_t FontSize,
                  uint16_t ForeColor, uint16_t BgColor);
void LCD_ShowString(uint16_t X, uint16_t Y, char *String, uint8_t FontSize,
                    uint16_t ForeColor, uint16_t BgColor);
void LCD_ShowNum(uint16_t X, uint16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize,
                 uint16_t ForeColor, uint16_t BgColor);
void LCD_ShowFloatNum(uint16_t X, uint16_t Y, double Number, uint8_t IntLength,
                      uint8_t FraLength, uint8_t FontSize, uint16_t ForeColor, uint16_t BgColor);
void LCD_ShowImage(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, const uint16_t *Image);
void LCD_ShowImageRaw(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, const uint8_t *Data);
/* 整块推送 (只设一次窗口 + 一次大 DMA), 比 LCD_ShowImageRaw 快很多;
 * 仅支持不裁切的整宽推送, 否则返回 false, 调用方应退回逐行版本 */
bool LCD_ShowImageRawFull(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, const uint8_t *Data);
void LCD_Printf(uint16_t X, uint16_t Y, uint8_t FontSize,
                uint16_t ForeColor, uint16_t BgColor, char *format, ...);

#endif