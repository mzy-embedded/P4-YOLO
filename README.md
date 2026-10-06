# P4-YOLO — CSI 摄像头本地预览 + YOLO 检测框叠加

基于 **ESP32-P4**（WT9932P4X-TINY，芯片 v3.x）+ **SC2336 MIPI-CSI 摄像头** +
**ST7789 1.3寸 240x240** 屏的目标检测工程。摄像头画面直接显示在 SPI-LCD 上，
并在画面上叠加 YOLO 检测框与类别标签，全程不需要连接电脑。

## 硬件接线（SPI-LCD 7 脚屏，无 CS）

| LCD 引脚 | ESP32 GPIO |
|----------|-----------|
| SCL      | IO16      |
| SDA      | IO17      |
| RES      | IO18      |
| DC       | IO19      |
| BLK      | IO20      |

- SC2336 摄像头接到开发板 **MIPI-CSI** 接口；
- **FUSB** 口接电脑，用于烧录固件和查看串口日志；
- 板载按键 **IO35**（低电平有效）：短按循环切换屏幕旋转方向。

## 数据链路

```text
SC2336 (MIPI CSI, 1024x600 RAW8)
  → ISP → RGB565
  → 中心裁剪 600x600 → 最近邻降采样 240x240 → LCD_ShowImageRawFull → ST7789
  → 另取一份同几何副本做推理 → 检测框直接写进 RGB565 缓冲
```

## 工程结构

- `main/main.c`：应用入口。取帧 → 裁剪/降采样 → 推屏，并把检测框写进缓冲后整帧推送。
- `main/detector.cpp` / `detector.h`：esp-dl COCO 检测器的纯 C 封装（内部是 C++，
  用 `extern "C"` 暴露给主程序，`main.c` 无需感知 C++）。文件内维护**英文与中文两张
  80 类名表**，两表靠 `result_t.category` 下标关联，顺序必须完全一致。
- `components/BSP/LCD`：ST7789 SPI-LCD 驱动（`lcd.c/lcd.h` + ASCII 字库 `lcd_font.c`
  + GB2312 全量汉字字库 `lcd_font_gb2312.c`），支持屏幕旋转与多任务绘制互斥。
- `components/wt_bsp`：厂商板级支持包，负责 CSI、ISP 与外设初始化（含 IO35 按键）。
- `tools/gen_font.py` + `tools/HZK16`：中文字库生成脚本与 HZK16 点阵源；
  重新执行 `python tools/gen_font.py` 即可再生成 `components/BSP/LCD/lcd_font_gb2312.c`。
- `components/Middlewares`：第三方中间件目录，当前为空。
- `partitions-16MiB.csv`：16MB flash 分区表，factory 8MB（承载约 2.9MB 的 YOLO 模型）。
- `sdkconfig.defaults`：本工程唯一的一份配置片段，板级与芯片版本设置都在里面。

## 构建与烧录

需要 **ESP-IDF v6.1**（目标芯片 `esp32p4`）。本工程只支持 **WT9932P4X-TINY**（ESP32-P4 v3.x）；
它和 v1.x 的 WT9932P4-TINY 外设接线相同，但固件不能混用。

在工程根目录打开 **PowerShell**：

```powershell
.\build.ps1                  # 构建，产物 build\P4-YOLO.bin
.\build.ps1 menuconfig       # 打开配置菜单
.\build.ps1 fullclean        # 清理构建目录

.\flash.ps1                  # 烧录到 COM17
.\flash.ps1 -Port COM5       # 指定其它串口
.\flash.ps1 -EraseAll        # 首次或换固件时整片擦除后再烧
```

`build.ps1` 已封装本机所需的 ESP-IDF v6.1 环境（工具链 PATH、移除 Git Bash 的 `MSYSTEM`、
`ESP_ROM_ELF_DIR` 等），并固定使用仓库内的 `sdkconfig.defaults`。不用脚本时，
参考脚本里的环境变量手动调用 `idf.py` 即可。

## 运行现象

上电后屏幕显示摄像头实时画面（约 24.7 fps），画面上叠加红色检测框与**黄色中文类别标签**
（如「人 87」「自行车 62」），标签用 GB2312 字库以 16x16 渲染；贴到屏幕右边的框，
标签会自动整体左移，不会被裁掉。
推理单帧约 600ms（P4 无 NPU），所以**检测框约 1.6 Hz 更新**——每个框会连续显示若干帧，
视觉上是稳定的。

屏幕**底部有一条 16px 高的中文状态行**，黑底白字，实时显示：

```text
目标 3  24.7fps  612ms
```

分别是当前检测到的目标数、显示帧率、最近一次推理耗时。状态行由
`main.c` 的 `fb_draw_status()` 直接写入 RGB565 缓冲（与检测框同一套路），
不走 SPI 逐字刷新，因此不影响帧率。

串口（`lcd_preview` tag）形如：

```text
I (...) lcd_preview: Initializing Wireless-Tag BSP
I (...) wt_bsp_csi: CSI initialized successfully
I (...) lcd_preview: Preview started: CSI 1024x600 -> LCD 240x240
I (...) lcd_preview: Displayed 100 frames
I (...) lcd_preview: Detect round 5 -> 2 boxes
```

### 预期警告

以下警告属于正常现象，不代表功能异常：

```text
W (...) board: Display device not found at address: 0x28
W (...) board: Touch device not found at address: 0x55
```

BSP 会扫描共享 I2C 总线上的可选显示屏与触摸控制器。本工程用的是 SPI 屏，没接 DSI 屏和触摸，
所以这两条是预期的。只要摄像头在地址 `0x30` 被检测到，摄像头初始化就会继续。

ISP 在 1024 宽度下的自动白平衡统计窗口会被内部对齐到 1020（窗口宽度必须能被 5x5 网格整除），
这条 `ISP_AWB` 警告无害，应用已在 `app_main()` 里运行时屏蔽了该 tag；若在别处使用 BSP 仍可能看到。

## 说明

- 本工程已关闭 USB Device UVC（`CONFIG_WT_BSP_ENABLE_USB_DEVICE_UVC=n`），由
  `wt_bsp_csi_start()` 独占 CSI 采集设备。
- 屏幕颜色异常（如红蓝互换、整体偏色）时，把 `main/main.c` 里的 `PREVIEW_RGB565_SWAP` 改成 `0` 重试。
- **屏幕旋转**：板载按键 IO35 短按循环切换 0°/90°/180°/270°；上电默认方向由
  `components/BSP/LCD/lcd.h` 的 `LCD_ROTATION_INIT` 决定（本工程取 `0`，与引入旋转功能前完全一致，
  改成其它值会让摄像头画面跟着转）。旋转会连同画面一起转，切换时按键回调通过
  `LCD_Begin/End` 与预览任务的整帧推屏互斥，不会把画面撕开。
- `.devcontainer/` 是按统一规范放置的 ESP-IDF 容器模板（内置 xtensa QEMU），
  与本工程需要的 IDF v6.1 + riscv32 并不一致，仅供目录规范对齐，不保证开箱可用。
