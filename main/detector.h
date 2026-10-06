/**
 * @file detector.h
 * @brief 目标检测器的纯 C 接口 (实现为 C++, 见 detector.cpp)。
 *
 * 之所以单独隔一层: esp-dl 是 C++ API (std::list / dl:: 命名空间), 而主程序是 C。
 * 本头文件用 extern "C" 暴露纯 C 接口, main.c 无需感知 C++。
 *
 * 生活周期约定:
 *  - detector_init() 必须在**推理任务**里调用 (模型加载栈开销不确定), 不要在
 *    app_main 或 CSI 回调里调。
 *  - detector_init() 之前 detector_run() 一律返回 -1。
 */

#ifndef __DETECTOR_H
#define __DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 单次推理最多返回的框数 (结果按置信度降序)。 */
#define DETECTOR_MAX_RESULTS    (10)

/** 一个检测框。坐标是**相对传入图像**的像素坐标 (esp-dl 内部已完成反映射)。 */
typedef struct {
    int16_t x1;         /* 左上角 X */
    int16_t y1;         /* 左上角 Y */
    int16_t x2;         /* 右下角 X */
    int16_t y2;         /* 右下角 Y */
    uint8_t category;   /* 类别索引 (COCO 0-79) */
    float   score;      /* 置信度 0.0-1.0 */
} detector_box_t;

/**
 * @brief 构造检测器并加载模型。
 * @return 成功返回 0; 失败返回 -1。
 */
int detector_init(void);

/**
 * @brief 对一张 RGB565 图像做目标检测。
 *
 * @param[in]  src 图像数据, RGB565 小端存放, 行跨距 = w * 2, 长度 w * h * 2。
 * @param[in]  w   图像宽度 (像素)。
 * @param[in]  h   图像高度 (像素)。
 * @param[out] out 结果数组, 最多写入 max 个。
 * @param[in]  max out 的容量。
 * @return 实际写入的框数; 失败返回 -1。
 *
 * @note 内部对推理返回的 std::list 只做拷贝, 不会把引用泄给调用方。
 */
int detector_run(const uint8_t *src, int w, int h, detector_box_t *out, int max);

/**
 * @brief 取英文类别名。category 越界时返回 "obj"。
 */
const char *detector_class_name(int category);

/**
 * @brief 取中文类别名 (用于屏上检测框标签)。category 越界时返回 "目标"。
 * @note  中文名表与英文名表在 detector.cpp 里一一对应, 同为 80 项。
 */
const char *detector_class_name_cn(int category);

/** @brief 释放检测器。 */
void detector_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __DETECTOR_H */
