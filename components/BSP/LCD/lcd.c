/**
 ******************************************************************************
 * @file    lcd.c
 * @brief   1.3寸 LCD 显示驱动 (ST7789, 4线SPI, 240x240 RGB565)
 * @note    参考 stm32f103_st7789 工程 (M130T-240240-RGB-7), 8bit cmd + 8bit param
 *          基于 ESP-IDF esp_lcd_panel_io, 通过 SPI2_HOST 驱动
 *          分辨率 240x240, 16bit RGB565 (Big-Endian), 无全屏帧缓冲, 绘制即刷新
 *          2026-09-18 换屏: 1.47寸 172x320 -> 1.3寸 240x240
 *          关键差异: SPI 模式3(模式0全黑), GRAM偏移=(0,0), 无 CS
 *          2026-10-03 移植到 UVC_Camera (WT9932P4X-TINY), 用作 CSI 摄像头本地预览屏
 *          接线: SCL16/SDA17/RES18/DC19/BLK20, SPI 时钟 80MHz
 ******************************************************************************
 */

#include "lcd.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_spi.h"        /* IDF 6.x: SPI panel IO 已拆到独立头文件 */
#include "esp_log.h"
#include "lcd_font.h"              /* ASCII / 中文点阵字库 (字模数据, 随 LCD 目录存放) */
/* 注: 原 172x320 开机图的 p.h 已废弃 (1.3寸屏分辨率不符), 如需开机图按 240x240 重新生成 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"       /* LCD_Begin/End 的绘制互斥锁 */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>

static const char *TAG = "LCD";

/*============================================================================*/
/* SPI 总线配置                                                                 */
/*============================================================================*/
#define LCD_SPI_HOST        SPI2_HOST       /* ESP32-S3 SPI2 (FSPI) */
#define LCD_SPI_FREQ_HZ     (80 * 1000 * 1000)  /* SPI 时钟 80MHz (ESP32-S3 外设上限, 满屏纯传输 ~11.5ms) */
#define LCD_SPI_DMA_CH      SPI_DMA_CH_AUTO
/* 单次最大传输字节数。原来只有 1024 (够一行 480B), 但整屏 240x240x2 = 115200B:
 * 把它放大到整屏大小, 才能让 LCD_ShowImageRawFull() 一次 DMA 推完整屏,
 * 而不是被切成很多个小 SPI 事务。 */
#define LCD_MAX_TRANSFER_SZ (LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * 2)

/* 屏幕旋转 (MADCTL + GRAM 偏移)
 * MADCTL 位: MY=0x80 MX=0x40 MV=0x20; 240x240 正方形屏, 旋转只影响可视区落在 GRAM 哪一段。
 * 偏移规律(本 M130T 屏实测): 仅含垂直翻转位 MY 的方向需要 +80 —— ST7789 内部 GRAM 为
 * 240x320, MY 翻转后 240 行可视带落到另一端; MY 同时带 MV 时该偏移就落在列轴。
 * 注意索引 0 (0x00/0/0) 与移植前的 LCD_MADCTL_VALUE=0x00 + LCD_COL_OFFSET=0 完全等价,
 * 所以 LCD_ROTATION_INIT 取 0 时画面与改动前逐像素一致。
 * 若颜色 R/B 互换则置 BGR 位 0x08 (本模块 M130T 实测不需要)                    */
typedef struct
{
    uint8_t  Madctl;        /* 0x36 扫描方向 */
    uint16_t ColOffset;     /* CASET 列偏移  */
    uint16_t RowOffset;     /* RASET 行偏移  */
} LcdRotation_t;

static const LcdRotation_t s_rotations[] =
{
    { 0x00,   0,   0 },   /* 0: 0°   出厂竖屏         (无 MY, 无偏移) */
    { 0x60,   0,   0 },   /* 1: 90°  (顺时针)         (无 MY, 无偏移) */
    { 0xC0,   0,  80 },   /* 2: 180° (倒屏,    含 MY, 行偏80) */
    { 0xA0,  80,   0 },   /* 3: 270° (逆时针90°,含 MY, 列偏80) */
};
#define LCD_ROT_NUM         (sizeof(s_rotations) / sizeof(s_rotations[0]))
/* LCD_ROTATION_INIT 已在 lcd.h 定义 */

static uint16_t s_col_offset = 0;             /* 当前旋转对应的 CASET 偏移     */
static uint16_t s_row_offset = 0;             /* 当前旋转对应的 RASET 偏移     */

static esp_lcd_panel_io_handle_t s_lcd_io = NULL;

/* 静态缓冲: 一行颜色缓冲(最大 240 像素) 与 字符缓冲(最大 16x16 全彩) */
static uint8_t  s_line_buf[LCD_PIXEL_WIDTH * 2];
static const uint8_t s_blank_glyph[32] = {0};   /* 字库未命中字符的空白字形(16x16) */

/* LCD 访问互斥锁 (LCD_Init 中确定性创建; 每个"完整绘制序列"整体取锁) */
static SemaphoreHandle_t s_lcd_mutex = NULL;

/*============================================================================*/
/* 底层通信 (esp_lcd_panel_io)                                                 */
/*============================================================================*/

/**
 * @brief  发送命令(可带参数字节)
 * @param  Cmd   命令码
 * @param  Param 参数字节指针 (可为 NULL)
 * @param  Len   参数字节数
 */
static void LCD_WriteCmdParam(uint8_t Cmd, const uint8_t *Param, uint16_t Len)
{
    esp_lcd_panel_io_tx_param(s_lcd_io, Cmd, Param, Len);
}

static void LCD_WriteCmd(uint8_t Cmd)
{
    LCD_WriteCmdParam(Cmd, NULL, 0);
}

/*============================================================================*/
/* 绘制互斥 (多任务各自把一整段绘制序列用 Begin/End 包住)                        */
/*============================================================================*/

/**
 * @brief  取得 LCD 访问权 (每个完整绘制序列调用一次)
 * @return 1 = 已取到锁, 可以绘制; 0 = 锁忙(超时), 调用方应跳过本次绘制
 * @note   取锁带超时 (FreeRTOS 推荐): 持锁任务异常挂起时本任务超时返回 0,
 *         调用方跳过本次绘制, 避免整个设备死锁冻结需复位。
 */
int LCD_Begin(void)
{
    if (s_lcd_mutex == NULL)
        return 1;
    if (xSemaphoreTake(s_lcd_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        return 1;

    /* 诊断: 打印互斥锁的当前持有者 (FreeRTOS xSemaphoreGetMutexHolder) */
    {
        TaskHandle_t holder = xSemaphoreGetMutexHolder(s_lcd_mutex);
        ESP_LOGW(TAG, "LCD lock busy, holder=%s waiter=%s",
                 holder ? pcTaskGetName(holder) : "???",
                 pcTaskGetName(NULL));
    }
    return 0;
}

/** @brief 释放 LCD 访问权 (与 LCD_Begin 配对) */
void LCD_End(void)
{
    if (s_lcd_mutex != NULL)
    {
        xSemaphoreGive(s_lcd_mutex);
    }
}

/**
 * @brief  切换屏幕旋转方向
 * @param  Index 旋转表索引: 0=0°, 1=90°, 2=180°, 3=270°
 * @note   同时更新 CASET/RASET 的 GRAM 偏移; 每帧绘制前调用即可。
 *         应在 LCD_Begin/End 保护下调用, 避免与整帧推屏的窗口命令交错。
 */
void LCD_SetRotation(uint8_t Index)
{
    if (Index >= LCD_ROT_NUM)
        return;
    s_col_offset = s_rotations[Index].ColOffset;
    s_row_offset = s_rotations[Index].RowOffset;
    LCD_WriteCmdParam(0x36, &s_rotations[Index].Madctl, 1);
}

/*============================================================================*/
/* 窗口/像素区域                                                               */
/*============================================================================*/

/**
 * @brief  设置显存读写窗口 (CASET + RASET, 240x240 范围内手动设定)
 */
static void LCD_SetWindow(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1)
{
    uint8_t param[4];

    X0 += s_col_offset;                     /* 列偏移: 定位到面板可见列   */
    X1 += s_col_offset;
    param[0] = X0 >> 8;                     /* CASET: 列地址           */
    param[1] = X0 & 0xFF;
    param[2] = X1 >> 8;
    param[3] = X1 & 0xFF;
    LCD_WriteCmdParam(0x2A, param, 4);

    Y0 += s_row_offset;                     /* 行偏移: 含 MY 的旋转方向下移80 */
    Y1 += s_row_offset;
    param[0] = Y0 >> 8;                     /* RASET: 行地址 0~319       */
    param[1] = Y0 & 0xFF;
    param[2] = Y1 >> 8;
    param[3] = Y1 & 0xFF;
    LCD_WriteCmdParam(0x2B, param, 4);

    LCD_WriteCmd(0x2C);                     /* RAMWR: 准备写显存        */
}

/**
 * @brief  刷新一行像素到屏幕 (逐行拉窗口, 避免整区块地址回绕问题)
 * @param  X0,Y0  起点坐标
 * @param  W      行宽(像素)
 * @param  Data   这一行像素的 RGB565 大端字节流, 长度 W*2
 */
static void LCD_FlushRow(uint16_t X0, uint16_t Y0, uint16_t W, const uint8_t *Data)
{
    LCD_SetWindow(X0, Y0, X0 + W - 1, Y0);
    /* -1 表示不再重复发命令 (SetWindow 已发 RAMWR 0x2C) */
    esp_lcd_panel_io_tx_color(s_lcd_io, -1, Data, W * 2);
}

/*============================================================================*/
/* 初始化                                                                      */
/*============================================================================*/

void LCD_Init(void)
{
    gpio_config_t gpio_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    /* --- 控制引脚初始化: DC / RST / BL --- */
    gpio_cfg.pin_bit_mask = (1ULL << LCD_DC_GPIO_PIN) | (1ULL << LCD_RST_GPIO_PIN);
    if (LCD_BL_GPIO_PIN >= 0)
        gpio_cfg.pin_bit_mask |= (1ULL << LCD_BL_GPIO_PIN);
    gpio_config(&gpio_cfg);

    /* --- 硬件复位: RST 低电平 ≥10us 后拉高 --- */
    gpio_set_level(LCD_RST_GPIO_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(LCD_RST_GPIO_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));         /* 复位完成后等待上电稳定  */

    /* --- SPI 总线初始化 (写-only, 无 MISO) --- */
    spi_bus_config_t buscfg = {
        .sclk_io_num     = LCD_CLK_GPIO_PIN,
        .mosi_io_num     = LCD_MOSI_GPIO_PIN,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_MAX_TRANSFER_SZ,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, LCD_SPI_DMA_CH));

    /* --- 挂载 ST7789 (esp_lcd 面板 IO, SPI 模式3, 8bit命令/数据) ---
     * ★ spi_mode=3 是本屏(M130T-240240-RGB-7)点亮的关键: 参考工程实测
     *   用模式0 会一直全黑, 只有模式3 (CPOL=1, CPHA=1) 能出图                   */
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num        = LCD_DC_GPIO_PIN,
        .cs_gpio_num        = -1,               /* 7脚屏无 CS 引脚, 板内已拉低 */
        .pclk_hz            = LCD_SPI_FREQ_HZ,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 8,
        .spi_mode           = 3,
        .trans_queue_depth  = 10,
        .on_color_trans_done = NULL,
        .user_ctx           = NULL,
        /* 仅当颜色缓冲位于 PSRAM 时生效: 让 SPI DMA 直读 PSRAM,
         * 避免驱动内部再分配 DMA 中转缓冲 (AUTO_PREVIEW 的降级路径会用到) */
        .flags = {
            .psram_dma_direct = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &s_lcd_io));

    /* --- ST7789 初始化序列 (针对 240x240 1.3寸 M130T 屏, 参考 stm32f103_st7789) --- */
    LCD_WriteCmd(0x01);                     /* 软复位 */
    vTaskDelay(pdMS_TO_TICKS(150));
    LCD_WriteCmd(0x11);                     /* 退出睡眠模式 */
    vTaskDelay(pdMS_TO_TICKS(120));

    {
        /* 1.3寸屏 Gamma 表 (与 1.47寸不同, 来自参考工程实测) */
        static const uint8_t gamma_p[14] =
            { 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23 };
        static const uint8_t gamma_n[14] =
            { 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23 };
        uint8_t param[16];

        LCD_SetRotation(LCD_ROTATION_INIT);                         /* 扫描方向(旋转表) */
        param[0] = 0x05;
        LCD_WriteCmdParam(0x3A, param, 1);                          /* 16bit/RGB565 */

        param[0] = 0x0C; param[1] = 0x0C; param[2] = 0x00;          /* PORCTRL: porch 时序 */
        param[3] = 0x33; param[4] = 0x33;
        LCD_WriteCmdParam(0xB2, param, 5);
        param[0] = 0x35;
        LCD_WriteCmdParam(0xB7, param, 1);                          /* GCTRL    */
        param[0] = 0x19;
        LCD_WriteCmdParam(0xBB, param, 1);                          /* VCOMS (1.47寸用 0x35) */
        param[0] = 0x2C;
        LCD_WriteCmdParam(0xC0, param, 1);                          /* LCMCTRL  */
        param[0] = 0x01;
        LCD_WriteCmdParam(0xC2, param, 1);                          /* VDVVRHEN */
        param[0] = 0x12;
        LCD_WriteCmdParam(0xC3, param, 1);                          /* VRHS  (1.47寸用 0x13) */
        param[0] = 0x20;
        LCD_WriteCmdParam(0xC4, param, 1);                          /* VDVS     */
        param[0] = 0x0F;
        LCD_WriteCmdParam(0xC6, param, 1);                          /* FRCTRL2  */
        param[0] = 0xA4; param[1] = 0xA1;
        LCD_WriteCmdParam(0xD0, param, 2);                          /* PWCTRL1  */

        memcpy(param, gamma_p, 14);
        LCD_WriteCmdParam(0xE0, param, 14);                         /* 正Gamma  */
        memcpy(param, gamma_n, 14);
        LCD_WriteCmdParam(0xE1, param, 14);                         /* 负Gamma  */
    }

    LCD_WriteCmd(0x21);                     /* 反显开启 (IPS 面板必须开, 否则负片) */
    LCD_WriteCmd(0x13);                     /* 正常显示模式 */
    LCD_WriteCmd(0x29);                     /* 开显示 */

    /* 背光点亮 */
    if (LCD_BL_GPIO_PIN >= 0)
        gpio_set_level(LCD_BL_GPIO_PIN, 1);

    LCD_Clear(LCD_COLOR_BLACK);

    /* LCD 访问互斥锁: 在 app_main 单线程阶段确定性创建 (不用惰性初始化,
     * 避免两个任务并发首次调用 LCD_Begin 时竞态产生重复 mutex -> 互斥失效
     * 后双任务并发写 SPI 造成偶发死锁/卡死, 需手动复位才恢复) */
    if (s_lcd_mutex == NULL)
    {
        s_lcd_mutex = xSemaphoreCreateMutex();
    }
    ESP_LOGI(TAG, "ST7789 240x240 LCD init OK");
}

/*============================================================================*/
/* 清屏 / 矩形填充                                                             */
/*============================================================================*/

void LCD_Clear(uint16_t Color)
{
    LCD_FillRect(0, 0, LCD_PIXEL_WIDTH - 1, LCD_PIXEL_HEIGHT - 1, Color);
}

void LCD_FillRect(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color)
{
    uint16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
    uint16_t w, h;
    size_t   i;

    if (x0 > x1) { uint16_t t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { uint16_t t = y0; y0 = y1; y1 = t; }
    if (x1 >= LCD_PIXEL_WIDTH)  x1 = LCD_PIXEL_WIDTH - 1;
    if (y1 >= LCD_PIXEL_HEIGHT) y1 = LCD_PIXEL_HEIGHT - 1;

    w = x1 - x0 + 1;
    h = y1 - y0 + 1;

    /* 构造一行颜色数据 (RGB565 大端) */
    for (i = 0; i < w; i++)
    {
        s_line_buf[i * 2]     = Color >> 8;
        s_line_buf[i * 2 + 1] = Color & 0xFF;
    }

    for (uint16_t y = 0; y < h; y++)
        LCD_FlushRow(x0, y0 + y, w, s_line_buf);
}

/*============================================================================*/
/* 绘图                                                                        */
/*============================================================================*/

void LCD_DrawPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    uint8_t buf[2];
    if (X >= LCD_PIXEL_WIDTH || Y >= LCD_PIXEL_HEIGHT) return;
    buf[0] = Color >> 8;
    buf[1] = Color & 0xFF;
    LCD_FlushRow(X, Y, 1, buf);
}

void LCD_DrawLine(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color)
{
    int16_t x = X0, y = Y0;
    int16_t dx = (X1 > X0) ? (X1 - X0) : (X0 - X1);
    int16_t dy = (Y1 > Y0) ? (Y1 - Y0) : (Y0 - Y1);
    int16_t sx = (X0 < X1) ? 1 : -1;
    int16_t sy = (Y0 < Y1) ? 1 : -1;
    int16_t err = dx - dy, e2;

    while (1)
    {
        LCD_DrawPoint(x, y, Color);
        if (x == X1 && y == Y1) break;
        e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
}

void LCD_DrawRect(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Color)
{
    LCD_DrawLine(X0, Y0, X1, Y0, Color);
    LCD_DrawLine(X0, Y0, X0, Y1, Color);
    LCD_DrawLine(X1, Y0, X1, Y1, Color);
    LCD_DrawLine(X0, Y1, X1, Y1, Color);
}

void LCD_DrawCircle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Color)
{
    int16_t x, y, d;
    if (Radius == 0) { LCD_DrawPoint(X, Y, Color); return; }
    d = 1 - Radius; x = 0; y = Radius;

    LCD_DrawPoint(X + x, Y + y, Color); LCD_DrawPoint(X - x, Y - y, Color);
    LCD_DrawPoint(X + y, Y + x, Color); LCD_DrawPoint(X - y, Y - x, Color);

    while (x < y)
    {
        x++;
        if (d < 0) d += 2 * x + 1;
        else       { y--; d += 2 * (x - y) + 1; }

        LCD_DrawPoint(X + x, Y + y, Color); LCD_DrawPoint(X + y, Y + x, Color);
        LCD_DrawPoint(X - x, Y - y, Color); LCD_DrawPoint(X - y, Y - x, Color);
        LCD_DrawPoint(X + x, Y - y, Color); LCD_DrawPoint(X + y, Y - x, Color);
        LCD_DrawPoint(X - x, Y + y, Color); LCD_DrawPoint(X - y, Y + x, Color);
    }
}

void LCD_FillCircle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Color)
{
    int16_t x, y, d, j;
    if (Radius == 0) { LCD_DrawPoint(X, Y, Color); return; }
    d = 1 - Radius; x = 0; y = Radius;

    for (j = -y; j <= y; j++)
        LCD_DrawPoint(X, Y + j, Color);

    while (x < y)
    {
        x++;
        if (d < 0) d += 2 * x + 1;
        else       { y--; d += 2 * (x - y) + 1; }

        for (j = -y; j <= y; j++) { LCD_DrawPoint(X + x, Y + j, Color); LCD_DrawPoint(X - x, Y + j, Color); }
        for (j = -x; j <= x; j++) { LCD_DrawPoint(X + y, Y + j, Color); LCD_DrawPoint(X - y, Y + j, Color); }
    }
}

/*============================================================================*/
/* 文本显示                                                                    */
/*============================================================================*/

/**
 * @brief  在指定区域绘制一个位图 (内置逐像素字库点阵)
 * @param  X,Y      左上角坐标
 * @param  BmpW     位图宽(像素)
 * @param  BmpH     位图高(像素)
 * @param  Bitmap   单色字模数据 (每行高位在前, 行字节对齐到 BmpW)
 * @param  ForeColor 前景色 (点=1)
 * @param  BgColor   背景色 (点=0)
 */
static void LCD_ShowBitmap(uint16_t X, uint16_t Y, uint16_t BmpW, uint16_t BmpH,
                           const uint8_t *Bitmap, uint16_t ForeColor, uint16_t BgColor)
{
    /* 注意: 复用 OLED 字库, 其布局为"列式" —
     * 每 8 行为一页, 页内每列 1 个字节, 字节的 bit0=该列顶行, bit7=该列底行 */
    static uint8_t s_img_buf[16 * 16 * 2];  /* 最大 16x16 全彩缓冲 */
    uint32_t p = 0;

    for (uint16_t j = 0; j < BmpH; j++)
    {
        uint16_t ByteBase = (j / 8) * BmpW;  /* 所在页的首字节 */
        uint8_t  RowBit   = j % 8;           /* 该行在字节内的位 */
        for (uint16_t i = 0; i < BmpW; i++)
        {
            uint8_t Bit = (Bitmap[ByteBase + i] >> RowBit) & 0x01;
            uint16_t Col = Bit ? ForeColor : BgColor;
            s_img_buf[p++] = Col >> 8;
            s_img_buf[p++] = Col & 0xFF;
        }
    }
    /* 逐行刷新 (每行 2*BmpW 字节, 1 行高度窗口) */
    for (uint16_t j = 0; j < BmpH; j++)
        LCD_FlushRow(X, Y + j, BmpW, s_img_buf + (size_t)j * BmpW * 2);
}

void LCD_ShowChar(uint16_t X, uint16_t Y, char Char, uint8_t FontSize,
                  uint16_t ForeColor, uint16_t BgColor)
{
    if (FontSize == LCD_8X16)
        LCD_ShowBitmap(X, Y, 8, 16, LCD_F8x16[Char - ' '], ForeColor, BgColor);
    else if (FontSize == LCD_6X8)
        LCD_ShowBitmap(X, Y, 6, 8, LCD_F6x8[Char - ' '], ForeColor, BgColor);
}

/* ---- 辅助: UTF-8 / GB2312 多字节字符解析 (与 OLED 驱动一致) ---- */
void LCD_ShowString(uint16_t X, uint16_t Y, char *String, uint8_t FontSize,
                    uint16_t ForeColor, uint16_t BgColor)
{
    uint16_t i = 0;
    uint16_t XOffset = 0;
    char SingleChar[5];
    uint8_t CharLength = 0;

    while (String[i] != '\0')
    {
#ifdef LCD_CHARSET_UTF8
        if ((String[i] & 0x80) == 0x00)
        {
            CharLength = 1; SingleChar[0] = String[i++]; SingleChar[1] = '\0';
        }
        else if ((String[i] & 0xE0) == 0xC0)
        {
            CharLength = 2; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; SingleChar[2] = '\0';
        }
        else if ((String[i] & 0xF0) == 0xE0)
        {
            CharLength = 3; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; if (String[i]=='\0') break;
            SingleChar[2] = String[i++]; SingleChar[3] = '\0';
        }
        else if ((String[i] & 0xF8) == 0xF0)
        {
            CharLength = 4; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; if (String[i]=='\0') break;
            SingleChar[2] = String[i++]; if (String[i]=='\0') break;
            SingleChar[3] = String[i++]; SingleChar[4] = '\0';
        }
        else { i++; continue; }
#endif

#ifdef LCD_CHARSET_GB2312
        if ((String[i] & 0x80) == 0x00)
        {
            CharLength = 1; SingleChar[0] = String[i++]; SingleChar[1] = '\0';
        }
        else
        {
            CharLength = 2; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; SingleChar[2] = '\0';
        }
#endif

        if (CharLength == 1)
        {
            LCD_ShowChar(X + XOffset, Y, SingleChar[0], FontSize, ForeColor, BgColor);
            XOffset += FontSize;
        }
        else
        {
            /* 汉字取模: 字表按 UTF-8 字节序二分查找 (lcd_font_gb2312.c)
             * 原实现是线性扫描, 全量字库 7365 字时每字符要比较几千次;
             * 二分查找把最坏情况降到 ~13 次 strcmp。 */
            const uint8_t *pFont = LCD_FontFind(SingleChar);

            if (FontSize == LCD_8X16)
            {
                /* 未命中(字库外字符)显示空白字形, 避免越界 */
                LCD_ShowBitmap(X + XOffset, Y, 16, 16,
                               pFont ? pFont : s_blank_glyph, ForeColor, BgColor);
                XOffset += 16;
            }
            else if (FontSize == LCD_6X8)
            {
                LCD_ShowChar(X + XOffset, Y, '?', LCD_6X8, ForeColor, BgColor);
                XOffset += LCD_6X8;
            }
        }
    }
}

/* ---- 辅助: 求幂 ---- */
static uint32_t LCD_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void LCD_ShowNum(uint16_t X, uint16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize,
                 uint16_t ForeColor, uint16_t BgColor)
{
    for (uint8_t i = 0; i < Length; i++)
        LCD_ShowChar(X + i * FontSize, Y,
                     Number / LCD_Pow(10, Length - i - 1) % 10 + '0',
                     FontSize, ForeColor, BgColor);
}

void LCD_ShowFloatNum(uint16_t X, uint16_t Y, double Number, uint8_t IntLength,
                      uint8_t FraLength, uint8_t FontSize, uint16_t ForeColor, uint16_t BgColor)
{
    uint32_t IntNum, FraNum, PowNum;

    if (Number >= 0) LCD_ShowChar(X, Y, '+', FontSize, ForeColor, BgColor);
    else             { LCD_ShowChar(X, Y, '-', FontSize, ForeColor, BgColor); Number = -Number; }

    IntNum = (uint32_t)Number;
    Number -= IntNum;
    PowNum = LCD_Pow(10, FraLength);
    FraNum = (uint32_t)(Number * PowNum + 0.5);
    IntNum += FraNum / PowNum;

    LCD_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize, ForeColor, BgColor);
    LCD_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize, ForeColor, BgColor);
    LCD_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize, ForeColor, BgColor);
}

/* 全彩图像直写: Image 为 RGB565 数组 (w*h 个), 区域逐行刷新 */
void LCD_ShowImage(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, const uint16_t *Image)
{
    uint16_t DrawW = (X + Width  < LCD_PIXEL_WIDTH)  ? Width  : (LCD_PIXEL_WIDTH  - X);
    uint16_t DrawH = (Y + Height < LCD_PIXEL_HEIGHT) ? Height : (LCD_PIXEL_HEIGHT - Y);

    for (uint16_t j = 0; j < DrawH; j++)
    {
        /* 行内转成大端字节流 (RGB565 高字节在前) */
        for (uint16_t i = 0; i < DrawW; i++)
        {
            s_line_buf[i * 2]     = (uint8_t)(Image[(size_t)j * Width + i] >> 8);
            s_line_buf[i * 2 + 1] = (uint8_t)(Image[(size_t)j * Width + i] & 0xFF);
        }
        LCD_FlushRow(X, Y + j, DrawW, s_line_buf);
    }
}

/* 全彩图像直写(Image2Lcd 字节流): Data 为 RGB565 大端字节流
 * (每像素 2 字节, 高字节在前, 行优先), 兼容 Image2Lcd 生成的 C 数组 */
void LCD_ShowImageRaw(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height,
                      const uint8_t *Data)
{
    if (X >= LCD_PIXEL_WIDTH || Y >= LCD_PIXEL_HEIGHT) return;
    uint16_t DrawW = (X + Width  < LCD_PIXEL_WIDTH ) ? Width  : (LCD_PIXEL_WIDTH  - X);
    uint16_t DrawH = (Y + Height < LCD_PIXEL_HEIGHT) ? Height : (LCD_PIXEL_HEIGHT - Y);

    for (uint16_t j = 0; j < DrawH; j++)
        LCD_FlushRow(X, Y + j, DrawW, Data + (size_t)j * Width * 2);
}

/**
 * @brief 整块推送: 只设一次窗口, 数据一次性交给 DMA。
 *
 * 与 LCD_ShowImageRaw 的区别是性能: 后者逐行调用, 每行都要发 CASET+RASET+RAMWR
 * (轮询事务, 会等队列排空) 再做一次传输, 240 行就是 720 个命令事务加 240 次传输,
 * 全部串行。本函数只发一次窗口命令 + 一次大传输, 由 esp_lcd 内部切块并用
 * CS 保持连续, 大幅降低 CPU 与 SPI 事务开销。
 *
 * @note 只支持不裁切的整宽推送 (DrawW 必须等于 Width); 需要裁切时返回 false。
 * @return true 表示已按整块方式推送; false 表示调用方应退回逐行版本。
 */
bool LCD_ShowImageRawFull(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height,
                          const uint8_t *Data)
{
    if (X >= LCD_PIXEL_WIDTH || Y >= LCD_PIXEL_HEIGHT) return false;

    uint16_t DrawW = (X + Width  < LCD_PIXEL_WIDTH ) ? Width  : (LCD_PIXEL_WIDTH  - X);
    uint16_t DrawH = (Y + Height < LCD_PIXEL_HEIGHT) ? Height : (LCD_PIXEL_HEIGHT - Y);

    /* 宽度被裁切时, 源数据的行跨距与窗口宽度不一致, 整块推送会错位。 */
    if (DrawW != Width || DrawH != Height) {
        return false;
    }

    LCD_SetWindow(X, Y, (uint16_t)(X + DrawW - 1), (uint16_t)(Y + DrawH - 1));
    /* cmd = -1: 窗口命令已在上面发出, 这里只送像素数据 */
    esp_lcd_panel_io_tx_color(s_lcd_io, -1, Data, (size_t)DrawW * DrawH * 2);
    return true;
}

void LCD_Printf(uint16_t X, uint16_t Y, uint8_t FontSize,
                uint16_t ForeColor, uint16_t BgColor, char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    LCD_ShowString(X, Y, String, FontSize, ForeColor, BgColor);
}