/**
 * @file main.c
 * @brief CSI 摄像头 -> ST7789 SPI-LCD 本地预览 + YOLO 检测框叠加 (WT9932P4X-TINY)。
 *
 * 数据链路: SC2336 (MIPI CSI, 1024x600 RAW8) -> ISP -> RGB565
 *           -> 中心裁剪 600x600 -> 最近邻降采样到 240x240 -> LCD_ShowImageRaw
 *           -> 检测任务另取一份同几何的副本做推理 -> 把框画进 RGB565 缓冲
 *
 * 说明:
 *  - 本工程已关闭 USB Device UVC (CONFIG_WT_BSP_ENABLE_USB_DEVICE_UVC=n), 由
 *    wt_bsp_csi_start() 独占 CSI 采集设备, 不接电脑也能出画面。
 *  - 驱动组件位于 components/BSP/LCD (移植自 10_spilcd), 接线 SCL16/SDA17/RES18/DC19/BLK20。
 *  - 检测框【直接写进 RGB565 缓冲】而不是调 LCD_DrawRect: 后者走 LCD_DrawPoint →
 *    LCD_FlushRow, 每个像素要发 4 次 SPI 事务, 一个 240 宽的框约 3840 次/帧, 会把
 *    24.7fps 打到个位数。写缓冲只花内存带宽, 几乎零成本。
 *  - 推理单帧约 600ms (P4 无 NPU), 所以检测约 1.6fps、显示仍 24.7fps; 框以约
 *    1.6Hz 更新, 每个框连续显示若干帧, 视觉上是稳定的。
 *
 * @version 0.1
 * @date 2026-10-03
 *
 */

/* ==================== [Includes] ========================================== */

#include "wt_bsp.h"
#include "lcd.h"
#include "lcd_font.h"           /* LCD_F6x8 字模: 把类别标签渲染进缓冲 */
#include "detector.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"          /* 帧率统计与推理计时 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ==================== [Defines] =========================================== */

/**
 * V4L2_PIX_FMT_RGB565 == v4l2_fourcc('R','G','B','P') == 0x50424752。
 *
 * 这里刻意【自己定义】而不是 #include "linux/videodev2.h":
 * 该头文件来自托管组件 esp_video, 而 main 组件的依赖是在首次 cmake 配置时展开的,
 * 那时托管组件尚未解析, REQUIRES 里写 esp_video 会直接
 * "Failed to resolve component 'esp_video' required by component 'main'"
 * 而中断构建 (fullclean 后必然复现)。自定义这个稳定的 Linux ABI 常量可以彻底
 * 断开这层依赖, 不要"优化"成加回 include。
 */
#define PREVIEW_PIX_FMT_RGB565  (0x50424752u)

/** CSI 采集尺寸, 必须与 SC2336 出厂模式一致 (board_config.h 的 BOARD_CSI_WIDTH/HEIGHT)。 */
#define PREVIEW_SRC_W           (1024)
#define PREVIEW_SRC_H           (600)

/** 从源图中心裁出正方形, 避免 240x240 屏上画面被拉伸。 */
#define PREVIEW_CROP_W          (PREVIEW_SRC_H)
#define PREVIEW_CROP_H          (PREVIEW_SRC_H)
#define PREVIEW_CROP_X          ((PREVIEW_SRC_W - PREVIEW_CROP_W) / 2)

/** 目标尺寸 = 屏幕分辨率。 */
#define PREVIEW_DST_W           (LCD_PIXEL_WIDTH)
#define PREVIEW_DST_H           (LCD_PIXEL_HEIGHT)

#define PREVIEW_BUF_SIZE        (PREVIEW_DST_W * PREVIEW_DST_H * 2)

/**
 * 双缓冲: 一块供显示任务推屏, 另一块供取帧回调写入, 取不到空闲块时丢帧。
 * 缓冲放在内部 DMA 内存中, 避免 esp_lcd SPI 驱动为 PSRAM 缓冲额外中转。
 */
#define PREVIEW_BUF_COUNT       (2)

/**
 * 摄像头 ISP 输出的 RGB565 为小端存放 (低字节在前), ST7789 需要大端 (高字节在前)。
 * 若上电后画面颜色异常 (如红蓝互换、整体偏色), 把此宏改为 0 重试。
 */
#define PREVIEW_RGB565_SWAP     (1)

#define PREVIEW_TASK_STACK      (4096)
#define PREVIEW_TASK_PRIO       (4)

/** 每显示这么多帧打印一次统计, 用于观察实际帧率与丢帧情况。 */
#define PREVIEW_LOG_INTERVAL    (100)

/**
 * 临时性能诊断开关: 置 1 时跳过"显示降采样 + 推屏", 但**仍然填充检测缓冲**,
 * 用来测出推理耗时里有多少是被预览链路抢占的。
 *
 * 注意: 早先的版本是在回调开头就 return, 那样连检测缓冲也没人填, 推理根本
 * 跑不起来, 测不到任何数据 —— 要测就必须让检测这一路保持供帧。
 *
 * 置 1 时 LCD 不出图, 测完必须改回 0。
 */
#define PREVIEW_LOAD_TEST       (0)

/* ---- 目标检测 ---- */

/**
 * 检测任务的栈与优先级。
 * 优先级必须【低于】PREVIEW_TASK_PRIO: 单帧推理约 600ms 纯占 CPU (P4 无 NPU),
 * 优先级更低才能让 CSI 采集回调和推屏抢占它, 否则预览会卡死。
 */
#define DETECT_TASK_STACK       (16384)
#define DETECT_TASK_PRIO        (3)

/** 两轮推理之间的间隔, 给 CSI 采集和刷新结果留出空隙。 */
#define DETECT_INTERVAL_MS      (200)

/** 单次推理最多接收的框数。 */
#define DETECT_MAX_BOXES        (10)

/** 等待检测缓冲的超时。超时说明 CSI 没有出帧, 放弃这一轮。 */
#define DETECT_FRAME_TIMEOUT_MS (500)

/* ---- 屏幕文字 (中文状态行 / 检测框标签) ---- */

/** 一个字模格的尺寸。GB2312 字库只有 16x16 一种字号; ASCII 用等高的 LCD_F8x16。 */
#define CJK_FONT_W              (16)
#define CJK_FONT_H              (16)

/** 状态行高度 = 一个汉字的高度; 整条黑底, 文字白色。 */
#define STATUS_BAR_H            (CJK_FONT_H)

/** 状态行底色与字色。 */
#define STATUS_BAR_BG           (LCD_COLOR_BLACK)
#define STATUS_BAR_FG           (LCD_COLOR_WHITE)

/** 帧率统计窗口: 每满这么多微秒刷新一次 s_disp_fps_x10。 */
#define PREVIEW_FPS_WINDOW_US   (500000)

/* ==================== [Static Variables] ================================== */

static const char *TAG = "lcd_preview";

static uint8_t *s_frame_buf[PREVIEW_BUF_COUNT];
static volatile int s_pending = -1;     /* 已写完待显示 的缓冲索引, -1 表示无 */
static volatile int s_consuming = -1;   /* 正在推屏 的缓冲索引, -1 表示无   */
static SemaphoreHandle_t s_frame_sem;

/* ---- 目标检测 ---- */

/** 检测专用缓冲, 与显示缓冲同几何 (240x240 RGB565), 但【不做字节交换】。
 *  不交换是为了保持 CSI 的原始小端布局, 对应 DL_IMAGE_PIX_TYPE_RGB565LE,
 *  避免让 esp-dl 的输入被显示屏的大端怪癖污染。它不喂 SPI, 所以放 PSRAM。 */
static uint8_t *s_detect_buf;
static volatile bool s_detect_req;      /* 检测任务请求一帧 */
static volatile bool s_detect_ready;    /* 回调已填好, 待检测任务消费 */
static SemaphoreHandle_t s_detect_sem;

static SemaphoreHandle_t s_result_mutex;    /* 保护下面两个变量 */
static detector_box_t s_result[DETECT_MAX_BOXES];
static int s_result_count;

/* ---- 状态行数据源 (单写单读, 32 位对齐访问无需加锁) ---- */

static volatile uint32_t s_disp_fps_x10;    /* 显示帧率 x10,  由 preview_task 更新 */
static volatile uint32_t s_infer_ms;        /* 最近一次推理耗时 ms, 由 detect_task 更新 */

/* 当前屏幕旋转索引 (0..3), 由按键回调推进; app_main 里以 LCD_ROTATION_INIT 初始化。 */
static uint8_t s_rotation = LCD_ROTATION_INIT;

/* ==================== [Static Prototypes] ================================= */

static esp_err_t preview_buffers_alloc(void);
static int preview_pick_writable(void);
static void preview_scale_ex(const uint8_t *src, size_t len, uint8_t *dst, bool swap);
static void preview_frame_cb(uint8_t *buf, uint32_t width, uint32_t height, size_t len, void *user_data);
static void preview_task(void *arg);
static void detect_task(void *arg);
static void detect_overlay(uint8_t *fb);
static void fb_set_px(uint8_t *fb, int x, int y, uint16_t color);
static void fb_draw_rect(uint8_t *fb, int x0, int y0, int x1, int y1, uint16_t color);
static void fb_blit_1bpp(uint8_t *fb, int x, int y, int w, int h,
                         const uint8_t *bmp, uint16_t fg, uint16_t bg);
static void fb_draw_cjk(uint8_t *fb, int x, int y, const char *s, uint16_t fg, uint16_t bg);
static int  fb_text_width(const char *s);
static void fb_draw_status(uint8_t *fb);
static void button_event_cb(wt_bsp_button_t button, wt_bsp_button_event_t event, void *user_data);

/* ==================== [Global Functions] ================================== */

void app_main(void)
{
    /* ISP 在 1024 宽度下的 AWB 子窗口会对齐到 1020, 逐帧告警但无害, 这里只屏蔽该 tag。 */
    esp_log_level_set("ISP_AWB", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "Initializing Wireless-Tag BSP");
    ESP_ERROR_CHECK(wt_bsp_init());

    wt_bsp_csi_t csi = wt_bsp_get_csi();
    if (csi == NULL) {
        ESP_LOGE(TAG, "CSI camera is unavailable");
        return;
    }

    if (preview_buffers_alloc() != ESP_OK) {
        return;
    }

    s_frame_sem = xSemaphoreCreateBinary();
    s_detect_sem = xSemaphoreCreateBinary();
    s_result_mutex = xSemaphoreCreateMutex();
    if (s_frame_sem == NULL || s_detect_sem == NULL || s_result_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sync primitives");
        return;
    }

    LCD_Init();

    /* 按键回调必须在 LCD_Init 之后注册: wt_bsp_init() 里按键任务就已建好,
     * 若在 LCD 就绪前触发回调, LCD_SetRotation -> LCD_WriteCmdParam 会用到
     * 还是 NULL 的 s_lcd_io。LCD_Init 之后注册即可彻底避开这个窗口。 */
    wt_bsp_button_t btn = wt_bsp_get_button();
    if (btn != NULL) {
        ESP_ERROR_CHECK(wt_bsp_button_register_event_cb(btn, button_event_cb, NULL));
        ESP_LOGI(TAG, "Button (IO35) ready: click to cycle LCD rotation");
    } else {
        ESP_LOGW(TAG, "Button unavailable, rotation switch disabled");
    }

    /* 让 ISP 直接输出 RGB565: 与屏幕位深一致, 回调里只需降采样 + 字节序转换。 */
    ESP_ERROR_CHECK(wt_bsp_csi_set_pixel_format(csi, PREVIEW_PIX_FMT_RGB565));

    BaseType_t created = xTaskCreate(preview_task, "lcd_preview",
                                     PREVIEW_TASK_STACK, NULL,
                                     PREVIEW_TASK_PRIO, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create preview task");
        return;
    }

    /* 检测任务先于 CSI 启动创建: 此时回调还不会跑, 检测任务可以安全地做
     * 模型加载而不与取帧竞争。检测失败不影响预览 (任务会自行退出)。
     *
     * 绑定到 core 1: 预览 + CSI 采集在 core 0, 把 600ms 量级的推理隔离到另一个核,
     * 实测不隔离时推理从 0.5s 被拖到 2.6s。 */
    created = xTaskCreatePinnedToCore(detect_task, "detect",
                                      DETECT_TASK_STACK, NULL,
                                      DETECT_TASK_PRIO, NULL, 1);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detect task");
    }

    ESP_ERROR_CHECK(wt_bsp_csi_start(csi, preview_frame_cb, NULL));

    ESP_LOGI(TAG, "Preview started: CSI %dx%d -> LCD %dx%d",
             PREVIEW_SRC_W, PREVIEW_SRC_H, PREVIEW_DST_W, PREVIEW_DST_H);
}

/* ==================== [Static Functions] ================================== */

/**
 * @brief 分配两块预览缓冲: 优先内部 DMA 内存, 退化到 PSRAM。
 */
static esp_err_t preview_buffers_alloc(void)
{
    const size_t size = PREVIEW_BUF_SIZE;

    for (int i = 0; i < PREVIEW_BUF_COUNT; i++) {
        s_frame_buf[i] = heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (s_frame_buf[i] == NULL) {
            ESP_LOGW(TAG, "Internal DMA buffer %d unavailable, falling back to PSRAM", i);
            s_frame_buf[i] = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
        }
        if (s_frame_buf[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate preview buffer %d (%u bytes)",
                     i, (unsigned int)size);
            return ESP_ERR_NO_MEM;
        }
    }

    /* 检测缓冲走 PSRAM: 它只被 CPU 读写、不喂 SPI, 不需要 DMA/内部 RAM,
     * 把宝贵的内部 RAM 留给显示缓冲。 */
    s_detect_buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (s_detect_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate detect buffer (%u bytes)", (unsigned int)size);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief 选出一块既没等显示、也没在推屏的缓冲。
 * @return 缓冲索引; 两块都占用中时返回 -1 (调用方应丢帧)。
 */
static int preview_pick_writable(void)
{
    for (int i = 0; i < PREVIEW_BUF_COUNT; i++) {
        if (i != s_pending && i != s_consuming) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 中心裁剪 + 最近邻降采样, 可选 RGB565 字节序转换。
 *
 * 采样步长 = CROP/DST = 600/240 = 2.5, 用整数公式 (n * CROP) / DST 实现,
 * 不做浮点、不做插值, 避免占用 CSI 回调的关键路径。
 *
 * @param[in]  src CSI 原始帧 (RGB565), 按 len/height 推算行跨距以兼容行填充。
 * @param[in]  len 帧数据字节数。
 * @param[out] dst 目标缓冲, 长度 PREVIEW_BUF_SIZE。
 * @param[in]  swap true = 显示缓冲 (转换成大端供 ST7789); false = 检测缓冲 (保持原样)。
 */
static void preview_scale_ex(const uint8_t *src, size_t len, uint8_t *dst, bool swap)
{
    size_t stride = len / PREVIEW_SRC_H;
    if (stride < (size_t)PREVIEW_SRC_W * 2) {
        stride = (size_t)PREVIEW_SRC_W * 2;
    }

    for (int y = 0; y < PREVIEW_DST_H; y++) {
        const uint8_t *row = src +
            (size_t)((y * PREVIEW_CROP_H) / PREVIEW_DST_H) * stride;
        uint8_t *out = dst + (size_t)y * PREVIEW_DST_W * 2;

        for (int x = 0; x < PREVIEW_DST_W; x++) {
            const uint8_t *p = row +
                (size_t)(PREVIEW_CROP_X + (x * PREVIEW_CROP_W) / PREVIEW_DST_W) * 2;

            if (swap) {
                out[0] = p[1];
                out[1] = p[0];
            } else {
                out[0] = p[0];
                out[1] = p[1];
            }
            out += 2;
        }
    }
}

/**
 * @brief CSI 帧回调, 运行在 wt_bsp_csi 的采集任务里。
 *
 * 注意: 回调返回后驱动会立刻把该缓冲重新入队, 所以必须在回调内完成降采样。
 */
static void preview_frame_cb(uint8_t *buf, uint32_t width, uint32_t height,
                             size_t len, void *user_data)
{
    (void)user_data;

    if (buf == NULL || width != PREVIEW_SRC_W || height != PREVIEW_SRC_H) {
        return;
    }

#if !PREVIEW_LOAD_TEST
    int idx = preview_pick_writable();
    if (idx >= 0) {
        preview_scale_ex(buf, len, s_frame_buf[idx], PREVIEW_RGB565_SWAP);
        s_pending = idx;
        xSemaphoreGive(s_frame_sem);
    }
#endif

    /* 检测缓冲: 只在检测任务请求、且上一帧已被它取走时才填。
     * 推理绝不在这个回调里做 —— 回调返回后驱动立刻 QBUF 回收缓冲,
     * 而且它跑在栈只有 8192 的 CSI 采集任务里。 */
    if (s_detect_req && !s_detect_ready) {
        preview_scale_ex(buf, len, s_detect_buf, false);
        s_detect_ready = true;
        xSemaphoreGive(s_detect_sem);
    }
}

/**
 * @brief 显示任务: 取最新一帧推给 LCD, 并周期性打印帧率统计。
 */
static void preview_task(void *arg)
{
    (void)arg;

    uint32_t frames = 0;                        /* 本统计窗口内的帧数 */
    uint32_t total  = 0;                        /* 累计帧数 (打日志用) */
    int64_t  win_start = esp_timer_get_time();

    while (1) {
        if (xSemaphoreTake(s_frame_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int idx = s_pending;
        if (idx < 0) {
            continue;
        }

        s_consuming = idx;
        s_pending = -1;

        /* 把上一次的检测结果画进缓冲, 再整帧推送。
         * 画进缓冲只花内存带宽; 若改成 LCD_DrawRect 会变成几千次 SPI 事务。 */
        detect_overlay(s_frame_buf[idx]);

        /* 整帧推送与按键回调里的 LCD_SetRotation 都会发 SPI 命令, 用 LCD 互斥锁
         * 把两者隔开; 取不到锁说明按键正在切旋转, 丢掉这一帧即可。 */
        if (LCD_Begin()) {
            /* 整块推送: 一次窗口命令 + 一次大 DMA, 取代逐行的 720 个命令事务。
             * 失败 (被裁切) 时退回逐行版本。 */
            if (!LCD_ShowImageRawFull(0, 0, PREVIEW_DST_W, PREVIEW_DST_H, s_frame_buf[idx])) {
                LCD_ShowImageRaw(0, 0, PREVIEW_DST_W, PREVIEW_DST_H, s_frame_buf[idx]);
            }
            LCD_End();
        }

        s_consuming = -1;

        /* 每 500ms 刷新一次状态行的帧率 (用时间窗口而不是固定帧数,
         * 这样帧率变化时状态行能立刻跟上, 不必等 100 帧)。 */
        frames++;
        total++;
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - win_start;
        if (elapsed >= PREVIEW_FPS_WINDOW_US) {
            s_disp_fps_x10 = (uint32_t)((int64_t)frames * 10000000 / elapsed);
            frames = 0;
            win_start = now;
        }

        if (total >= PREVIEW_LOG_INTERVAL) {
            ESP_LOGI(TAG, "Displayed %u frames", (unsigned int)total);
            total = 0;
        }
    }
}

/**
 * @brief 检测任务: 周期性取一帧副本送推理, 结果发布到共享结构体。
 *
 * 优先级低于预览任务, 推理期间(约 600ms)被打断也不影响出图。
 * 注意 run() 期间【绝不持有结果互斥锁】, 否则预览任务会卡死丢帧。
 */
static void detect_task(void *arg)
{
    (void)arg;

    if (detector_init() != 0) {
        ESP_LOGE(TAG, "detector_init failed, detection disabled (preview keeps running)");
        vTaskDelete(NULL);
        return;
    }

    detector_box_t local[DETECT_MAX_BOXES];
    uint32_t rounds = 0;

    while (1) {
        s_detect_req = true;

        if (xSemaphoreTake(s_detect_sem, pdMS_TO_TICKS(DETECT_FRAME_TIMEOUT_MS)) == pdTRUE) {
            s_detect_req = false;

            /* 计时只包住推理本身: 取帧等待与发布结果都不算, 这样状态行上的
             * 耗时才是纯推理耗时。 */
            int64_t t_infer = esp_timer_get_time();
            int n = detector_run(s_detect_buf, PREVIEW_DST_W, PREVIEW_DST_H,
                                 local, DETECT_MAX_BOXES);
            s_infer_ms = (uint32_t)((esp_timer_get_time() - t_infer) / 1000);

            /* 到这里推理已结束、local 已填好, 才短暂地上锁发布。 */
            if (n >= 0) {
                xSemaphoreTake(s_result_mutex, portMAX_DELAY);
                memcpy(s_result, local, (size_t)n * sizeof(detector_box_t));
                s_result_count = n;
                xSemaphoreGive(s_result_mutex);
            }

            s_detect_ready = false;     /* 交还缓冲, 让回调填下一帧 */

            if ((++rounds % 5) == 0) {
                ESP_LOGI(TAG, "Detect round %u -> %d boxes", (unsigned int)rounds, n);
            }
        } else {
            s_detect_req = false;
            s_detect_ready = false;
            ESP_LOGW(TAG, "Detect frame timeout, CSI may not be streaming");
        }

        vTaskDelay(pdMS_TO_TICKS(DETECT_INTERVAL_MS));
    }
}

/**
 * @brief 往 RGB565 缓冲写一个像素。
 *
 * 显示缓冲在两种相机字节序下都最终是大端 (ST7789 要求高字节在前):
 * PREVIEW_RGB565_SWAP 控制的是"源是否小端", 转换后写进缓冲的一律是大端。
 */
static void fb_set_px(uint8_t *fb, int x, int y, uint16_t color)
{
    uint8_t *p = fb + ((size_t)y * PREVIEW_DST_W + (size_t)x) * 2;
    p[0] = (uint8_t)(color >> 8);
    p[1] = (uint8_t)(color & 0xFF);
}

/** @brief 在缓冲里画空心矩形 (与 LCD_DrawRect 的语义一致)。 */
static void fb_draw_rect(uint8_t *fb, int x0, int y0, int x1, int y1, uint16_t color)
{
    for (int x = x0; x <= x1; x++) {
        fb_set_px(fb, x, y0, color);
        fb_set_px(fb, x, y1, color);
    }
    for (int y = y0; y <= y1; y++) {
        fb_set_px(fb, x0, y, color);
        fb_set_px(fb, x1, y, color);
    }
}

/**
 * @brief 把 1bpp 字模渲染进 RGB565 缓冲 (列式分页, 与 lcd.c 的 LCD_ShowBitmap 同索引)。
 *
 * 字模每 8 行为一页: 页内每列 1 个字节, bit0 = 该列该页的最上面一行。
 * 这里复刻该索引写进缓冲, 而不是调 LCD_ShowBitmap —— 后者走 LCD_FlushRow,
 * 每个字形都要发一次窗口命令, 状态行每帧重画会把帧率拖下来。
 */
static void fb_blit_1bpp(uint8_t *fb, int x, int y, int w, int h,
                         const uint8_t *bmp, uint16_t fg, uint16_t bg)
{
    for (int j = 0; j < h; j++) {
        const int py = y + j;
        if (py < 0 || py >= PREVIEW_DST_H) {
            continue;
        }
        const int byte_base = (j / 8) * w;   /* 所在页的首字节 */
        const int row_bit   = j % 8;         /* 该行在字节内的位 */

        for (int i = 0; i < w; i++) {
            const int px = x + i;
            if (px < 0 || px >= PREVIEW_DST_W) {
                continue;
            }
            const uint8_t bit = (uint8_t)((bmp[byte_base + i] >> row_bit) & 0x01);
            fb_set_px(fb, px, py, bit ? fg : bg);
        }
    }
}

/**
 * @brief 把 UTF-8 字符串渲染进缓冲, 中英文混排。
 *
 * ASCII 走 LCD_F8x16 (8x16, 与汉字等高, 混排不会出现基线错位);
 * 多字节字符走 LCD_FontFind 从 GB2312 全量字库取 16x16 字模。
 * 字库外字符用空白字形占位, 保证步进宽度一致、后续字符不会串位。
 */
static void fb_draw_cjk(uint8_t *fb, int x, int y, const char *s, uint16_t fg, uint16_t bg)
{
    static const uint8_t blank[32] = {0};

    while (*s != '\0' && x < PREVIEW_DST_W) {
        const unsigned char c = (unsigned char)*s;
        const uint8_t *glyph;
        int w;

        if (c < 0x80) {
            glyph = (c >= ' ' && c <= '~') ? LCD_F8x16[c - ' '] : NULL;
            w = 8;
            s++;
        } else {
            /* 由 UTF-8 前导字节定长度; 末尾不完整就直接收尾, 避免读越界 */
            char single[5];
            const int len = ((c & 0xE0) == 0xC0) ? 2 :
                            ((c & 0xF0) == 0xE0) ? 3 :
                            ((c & 0xF8) == 0xF0) ? 4 : 0;
            if (len == 0) {
                s++;
                continue;
            }
            for (int i = 0; i < len; i++) {
                if (s[i] == '\0') {
                    return;
                }
                single[i] = s[i];
            }
            single[len] = '\0';
            glyph = LCD_FontFind(single);
            w = 16;
            s += len;
        }

        fb_blit_1bpp(fb, x, y, w, CJK_FONT_H, glyph ? glyph : blank, fg, bg);
        x += w;
    }
}

/**
 * @brief 预算 fb_draw_cjk 渲染该串占用的像素宽度。
 *
 * 步进规则必须与 fb_draw_cjk 保持一致: ASCII = 8px, 多字节汉字 = 16px。
 * 用来把贴到右边缘的检测框标签整体左移, 避免后半截被裁掉。
 */
static int fb_text_width(const char *s)
{
    int w = 0;

    while (*s != '\0') {
        const unsigned char c = (unsigned char)*s;
        if (c < 0x80) {
            w += 8;
            s++;
        } else {
            const int len = ((c & 0xE0) == 0xC0) ? 2 :
                            ((c & 0xF0) == 0xE0) ? 3 :
                            ((c & 0xF8) == 0xF0) ? 4 : 0;
            if (len == 0) {
                s++;
                continue;
            }
            w += CJK_FONT_W;
            s += len;
        }
    }
    return w;
}

/**
 * @brief 渲染屏幕底部的中文状态行: 目标数 / 显示帧率 / 推理耗时。
 */
static void fb_draw_status(uint8_t *fb)
{
    int n;

    xSemaphoreTake(s_result_mutex, portMAX_DELAY);
    n = s_result_count;
    xSemaphoreGive(s_result_mutex);

    /* 先把整条底色刷掉: 状态行是叠在实时画面上的, 不刷会拖影 */
    for (int y = PREVIEW_DST_H - STATUS_BAR_H; y < PREVIEW_DST_H; y++) {
        for (int x = 0; x < PREVIEW_DST_W; x++) {
            fb_set_px(fb, x, y, STATUS_BAR_BG);
        }
    }

    char status[48];
    snprintf(status, sizeof(status), "目标 %d  %u.%ufps  %ums",
             n, (unsigned int)(s_disp_fps_x10 / 10), (unsigned int)(s_disp_fps_x10 % 10),
             (unsigned int)s_infer_ms);

    fb_draw_cjk(fb, 2, PREVIEW_DST_H - STATUS_BAR_H, status, STATUS_BAR_FG, STATUS_BAR_BG);
}

/**
 * @brief 按键事件回调: 短按循环切换屏幕旋转方向。
 *
 * 回调跑在 wt_bsp 的按键任务里, 与 preview_task 的整帧推屏并发。
 * LCD_SetRotation 会发 0x36 命令, 必须和推屏的窗口命令互斥, 否则整帧会被撕开。
 */
static void button_event_cb(wt_bsp_button_t button, wt_bsp_button_event_t event, void *user_data)
{
    (void)button;
    (void)user_data;

    if (event != WT_BSP_BUTTON_EVENT_CLICK) {
        return;
    }

    if (!LCD_Begin()) {
        ESP_LOGW(TAG, "Rotation switch skipped: LCD busy");
        return;
    }

    /* 旋转表共 4 项 (lcd.c 的 s_rotations, 取值 0..3) */
    s_rotation = (uint8_t)((s_rotation + 1) % 4);
    LCD_SetRotation(s_rotation);
    LCD_End();

    ESP_LOGI(TAG, "LCD rotation -> %u (%u deg)",
             (unsigned int)s_rotation, (unsigned int)(s_rotation * 90));
}

/**
 * @brief 把最近一次的检测结果画到缓冲上 (框 + 类别标签 + 置信度)。
 *
 * 只被 preview_task 调用, 所以不需要为 LCD 竞争加锁; 但读结果时要拿 result 锁。
 */
static void detect_overlay(uint8_t *fb)
{
    detector_box_t local[DETECT_MAX_BOXES];
    int n;

    xSemaphoreTake(s_result_mutex, portMAX_DELAY);
    n = s_result_count;
    if (n > 0) {
        memcpy(local, s_result, (size_t)n * sizeof(detector_box_t));
    }
    xSemaphoreGive(s_result_mutex);

    for (int i = 0; i < n; i++) {
        int x0 = local[i].x1;
        int y0 = local[i].y1;
        int x1 = local[i].x2;
        int y1 = local[i].y2;

        /* 裁剪到缓冲范围 */
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > PREVIEW_DST_W - 1) x1 = PREVIEW_DST_W - 1;
        if (y1 > PREVIEW_DST_H - 1) y1 = PREVIEW_DST_H - 1;
        if (x1 <= x0 || y1 <= y0) {
            continue;
        }

        fb_draw_rect(fb, x0, y0, x1, y1, LCD_COLOR_RED);

        /* 中文类别名: 最长 5 字 (停车计时器), UTF-8 下 15 字节, 48 足够 */
        char label[48];
        snprintf(label, sizeof(label), "%s %d",
                 detector_class_name_cn(local[i].category),
                 (int)(local[i].score * 100));

        /* 标签放在框上方; 顶到边就挪到框内 (按 16px 字高算)。
         * 标签画在框底下会被状态行压住, 所以再往上贴一点没坏处。 */
        int ty = (y0 >= CJK_FONT_H) ? (y0 - CJK_FONT_H) : (y0 + 1);

        /* 贴右边框的框, 标签会伸出屏外; 整体左移让它完整可见。 */
        int lx = x0;
        int lw = fb_text_width(label);
        if (lx + lw > PREVIEW_DST_W) {
            lx = PREVIEW_DST_W - lw;
        }
        if (lx < 0) {
            lx = 0;
        }

        fb_draw_cjk(fb, lx, ty, label, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
    }

    /* 状态行最后画, 保证压在检测框之上始终可读。 */
    fb_draw_status(fb);
}
