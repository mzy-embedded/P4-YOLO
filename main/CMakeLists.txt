# REQUIRES 里的托管组件 (coco_detect / esp-dl) 之所以能解析, 是因为
# main/idf_component.yml 用【无条件】方式声明了它们 —— 组件管理器在首次 cmake
# 配置时就会下载, 同一遍里 REQUIRES 就能找到。反之若依赖写在 rules 里 (如 wt_bsp
# 对 esp_video 的做法), fullclean 后首遍 CONFIG 未生成, 组件不会下载,
# 这里就会 "Failed to resolve component" 而中断构建。
idf_component_register(SRCS "main.c"
                            "detector.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES wt_bsp
                                BSP
                                coco_detect
                                esp-dl)
