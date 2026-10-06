/**
 * @file detector.cpp
 * @brief esp-dl COCO 目标检测器的 C++ 实现 (对外是 detector.h 的纯 C 接口)。
 *
 * 模型: coco_detect 组件的 YOLO11n 320x320 int8 (COCODetect::YOLO11N_320_S8_V1)。
 *       输入 320x320、80 类 COCO、模型文件约 2.9MB, 按 CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA
 *       打包进 app 的 rodata。
 *
 * 输入约定: 接收任意尺寸的 RGB565 小端图, esp-dl 内部会 letterbox 缩放到 320x320
 *          (coco_detect.cpp 里已 enable_letterbox({114,114,114})), 所以喂 240x240
 *          的正方形不会产生长宽比失真; 返回的 box 坐标已映射回传入图尺寸。
 *
 * 注意: 本文件故意【不包含 lcd.h】—— 画框由纯 C 的 main.c 负责, 从根上绕开
 *       lcd.h 缺少 extern "C" 保护导致的链接问题。
 */

#include "detector.h"

#include "coco_detect.hpp"      /* COCODetect */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include <new>
#include <list>
#include <string>
#include <map>

static const char *TAG = "detector";

/** 与 result_t.category 一一对应的 COCO 80 类 (顺序不可随意改动)。 */
static const char *const s_coco_names[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

/**
 * 与 s_coco_names 一一对应的中文类别名 (同样 80 项)。
 *
 * 顺序必须与上面的英文表【完全一致】: 两表都靠 result_t.category 这个下标取值,
 * 任何一处顺序被改都会让标签张冠李戴。增删条目时务必两张表一起动。
 *
 * 最长 5 个汉字 (停车计时器 / 笔记本电脑)。屏上按 16x16 字模渲染, 单个框的标签
 * 最宽 5*16 + 空格8 + 三位置信度24 = 112px, 240 宽屏放得下; 贴右边框时
 * main.c 的 detect_overlay 会把标签整体左移, 避免被裁。
 */
static const char *const s_coco_names_cn[80] = {
    "人", "自行车", "汽车", "摩托车", "飞机", "公交车", "火车", "卡车",
    "船", "红绿灯", "消防栓", "停止标志", "停车计时器", "长椅",
    "鸟", "猫", "狗", "马", "羊", "牛", "大象", "熊", "斑马",
    "长颈鹿", "背包", "雨伞", "手提包", "领带", "行李箱", "飞盘",
    "滑雪板", "单板滑雪", "球", "风筝", "棒球棒", "棒球手套",
    "滑板", "冲浪板", "网球拍", "瓶子", "酒杯", "杯子",
    "叉子", "刀", "勺子", "碗", "香蕉", "苹果", "三明治", "橙子",
    "西兰花", "胡萝卜", "热狗", "披萨", "甜甜圈", "蛋糕", "椅子", "沙发",
    "盆栽", "床", "餐桌", "马桶", "电视", "笔记本电脑", "鼠标",
    "遥控器", "键盘", "手机", "微波炉", "烤箱", "烤面包机", "水槽",
    "冰箱", "书", "时钟", "花瓶", "剪刀", "泰迪熊", "吹风机",
    "牙刷"
};

static COCODetect *s_detect = nullptr;

/* ==================== [Global Functions] ================================== */

extern "C" int detector_init(void)
{
    if (s_detect != nullptr) {
        return 0;
    }

    ESP_LOGI(TAG, "Loading model, free PSRAM=%uK free internal=%uK",
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10),
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) >> 10));

    /* lazy_load=false: 在构造里就完成模型加载, 让加载失败立刻暴露在 init 阶段,
     * 而不是拖到第一次 run() 时才炸。加载本身开销大, 所以这个调用必须在
     * 栈充足的检测任务里 (detector_init 的约定见 detector.h)。 */
    s_detect = new (std::nothrow) COCODetect(COCODetect::YOLO11N_320_S8_V1, false);
    if (s_detect == nullptr) {
        ESP_LOGE(TAG, "new COCODetect failed");
        return -1;
    }

    /* DetectWrapper::run() 在模型为空时会空指针解引用, 所以这里必须先确认
     * 模型真的加载成功。get_raw_model(0) 返回内部 dl::Model*, 加载失败时为 null。 */
    dl::Model *raw = s_detect->get_raw_model(0);
    if (raw == nullptr) {
        ESP_LOGE(TAG, "Model failed to load. Check CONFIG_FLASH_COCO_DETECT_YOLO11N_320_S8_V1 "
                      "and CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA in menuconfig.");
        delete s_detect;
        s_detect = nullptr;
        return -1;
    }

    /* 打印模型真实输入形状: 用来确认加载的确实是 320 变体。
     * 实测单帧推理约 2.64s, 与官方 640 变体的 2.77s 接近、而 320 变体标称 0.6s,
     * 需要先排除"装错模型"这个可能。 */
    std::map<std::string, dl::TensorBase *> &inputs = raw->get_inputs();
    for (std::map<std::string, dl::TensorBase *>::iterator it = inputs.begin();
         it != inputs.end(); ++it) {
        std::string shape;
        for (size_t i = 0; i < it->second->shape.size(); i++) {
            if (i) {
                shape += "x";
            }
            shape += std::to_string(it->second->shape[i]);
        }
        ESP_LOGI(TAG, "model input '%s' shape = [%s]",
                 it->first.c_str(), shape.c_str());
    }

    ESP_LOGI(TAG, "COCODetect(YOLO11N_320_S8_V1) ready, free PSRAM=%uK",
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
    return 0;
}

extern "C" int detector_run(const uint8_t *src, int w, int h, detector_box_t *out, int max)
{
    if (s_detect == nullptr || src == NULL || out == NULL || max <= 0 || w <= 0 || h <= 0) {
        return -1;
    }

    dl::image::img_t img = {};
    img.data     = const_cast<uint8_t *>(src);
    img.width    = (uint16_t)w;
    img.height   = (uint16_t)h;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

    /* run() 返回的 list 引用指向后处理器内部成员, 下一次 run() 就会失效,
     * 所以下面立刻全部拷出, 绝不把引用传出本函数。 */
    int64_t t0 = esp_timer_get_time();
    std::list<dl::detect::result_t> &res = s_detect->run(img);
    int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;

    int n = 0;
    for (std::list<dl::detect::result_t>::iterator it = res.begin();
         it != res.end() && n < max; ++it) {
        if (it->box.size() < 4) {
            continue;               /* 只接受矩形框, 跳过形状异常的条目 */
        }
        out[n].category = (uint8_t)(it->category < 0 ? 0 : it->category);
        out[n].score    = it->score;
        out[n].x1 = (int16_t)it->box[0];
        out[n].y1 = (int16_t)it->box[1];
        out[n].x2 = (int16_t)it->box[2];
        out[n].y2 = (int16_t)it->box[3];
        ++n;
    }

    ESP_LOGI(TAG, "run(%dx%d) -> %d boxes in %lld ms", w, h, n, (long long)dt_ms);
    return n;
}

extern "C" const char *detector_class_name(int category)
{
    if (category >= 0 && category < 80) {
        return s_coco_names[category];
    }
    return "obj";
}

extern "C" const char *detector_class_name_cn(int category)
{
    if (category >= 0 && category < 80) {
        return s_coco_names_cn[category];
    }
    return "目标";
}

extern "C" void detector_deinit(void)
{
    /* 调用前必须确保没有并发的 detector_run()。 */
    delete s_detect;
    s_detect = nullptr;
}
