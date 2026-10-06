/**
    板级驱动组件（BSP）目录说明

    LCD      — ST7789 1.3寸 SPI-LCD 驱动
                 接线: SCL=IO16, SDA=IO17, RES=IO18, DC=IO19, BLK=IO20
                 用于把 CSI 摄像头画面整帧推屏，并提供画框 / 字符叠加接口。

                 文件分工:
                   lcd.c/lcd.h           驱动主体 (SPI 初始化、绘图、整帧推屏、
                                         LCD_SetRotation 屏幕旋转、LCD_Begin/End
                                         多任务绘制互斥)
                   lcd_font.c/lcd_font.h ASCII 点阵字库 (6x8 / 8x16) 与字模类型定义
                   lcd_font_gb2312.c     GB2312 全量汉字 16x16 点阵 (7365 字)，
                                         并提供 LCD_FontFind 二分查找。
                                         由 tools/gen_font.py 从 tools/HZK16 生成，
                                         勿手工编辑。

    说明: 厂商板级支持包 wt_bsp（摄像头 CSI、ISP、各外设初始化）位于
          components/wt_bsp，不在本组件内，但同属板级驱动范畴。
          本工程固定 WT9932P4X-TINY（ESP32-P4 v3.x），已移除其多板卡选板机制。
 */
