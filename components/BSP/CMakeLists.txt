# BSP 组件: 从 10_spilcd 移植的 ST7789 SPI-LCD 驱动
set(src_dirs
            LCD)

set(include_dirs
            LCD)

set(requires
            driver
            esp_lcd)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-error=missing-braces -Wno-error=misleading-indentation)
