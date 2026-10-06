# -*- coding: utf-8 -*-
"""
gen_font.py — 从标准 HZK16 点阵字库生成 ESP32 LCD 中文 16x16 点阵表
=====================================================================

输入 (本目录):
    HZK16       标准 16x16 点阵字库 (GB2312 区位码索引), 每字 32 字节

输出 (相对本脚本 ../components/BSP/LCD/lcd_font_gb2312.c):
    const ChineseCell_t LCD_CF16x16[] = { ... }
    - 覆盖全部有效 GB2312 字符 (符号区 01~09 + 汉字区 16~87, 共约 7400 字)
    - 按 UTF-8 字节序升序排列, 便于 lcd.c 用 strcmp 二分查找
    - 末尾保留哨兵项 { "", {0} } 作为查找边界
    - 索引为 UTF-8 十六进制转义字符串 (纯 ASCII 输出, 无编码风险)

字模布局 (输出格式与 lcd.c 的 LCD_ShowBitmap 读法一致):
    每字 32 字节, 列式: 前 16 字节 = 上 8 行, 后 16 字节 = 下 8 行
    每字节一列, 字节内 bit0 = 该列顶行, bit7 = 该列底行
    源 HZK16 为行式(bit7=左), 生成时经 hzk_to_lcd() 转置为列式
    GB2312 区位码读取偏移: ((hi-0xA1)*94 + (lo-0xA1))*32

字库来源: py-zhao/mplus_hzk_12 (M+ 字体派生, 许可 M+ / Unlicense 宽松协议)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_FILE = os.path.join(HERE, "HZK16")
OUT_FILE = os.path.normpath(os.path.join(HERE, "..", "components", "BSP", "LCD", "lcd_font_gb2312.c"))


def esc_utf8(s: bytes) -> str:
    """字节流转 C 十六进制转义字符串 (纯 ASCII)."""
    return "".join("\\x%02X" % b for b in s)


def glyph_bytes(g: bytes) -> str:
    """32 字节字形转 C 数组, 每 16 个一行."""
    rows = []
    for i in range(0, 32, 16):
        rows.append("            " + ", ".join("0x%02X" % b for b in g[i:i + 16]))
    return ",\n".join(rows)


def hzk_to_lcd(raw: bytes) -> bytes:
    """行式bit7=左(HZK16 .fnt) -> 列式bit0=顶(本驱动 LCD_ShowBitmap).

    HZK16 读: pixel(x,y) = (raw[y*2 + x//8] >> (7 - x%8)) & 1
    LCD 写 : out[x + (y//8)*16] 的 bit(y%8) = pixel(x,y)
    """
    out = bytearray(32)
    for y in range(16):
        for x in range(16):
            if (raw[y * 2 + x // 8] >> (7 - (x % 8))) & 1:
                out[x + (y // 8) * 16] |= 1 << (y % 8)
    return bytes(out)


def main() -> None:
    data = open(FONT_FILE, "rb").read()
    print("HZK16 大小:", len(data), "字节, 字位:", len(data) // 32)

    items = []          # (utf8 字节, 32字节字形)
    for hi in range(0xA1, 0xF8):            # 区 01~87 (0xA1~0xF7)
        for lo in range(0xA1, 0xFE):
            try:
                ch = bytes((hi, lo)).decode("gb2312")   # 有效 GB2312 字符?
            except UnicodeDecodeError:
                continue
            off = ((hi - 0xA1) * 94 + (lo - 0xA1)) * 32
            glyph = hzk_to_lcd(data[off:off + 32])   # 行式 -> 列式, 对齐驱动取模
            if not any(glyph):              # 跳过空字形
                continue
            items.append((ch.encode("utf-8"), glyph))

    items.sort(key=lambda t: t[0])          # UTF-8 字节序 -> strcmp 升序
    print("有效字符:", len(items))

    # ---- 生成 C 源文件 (纯 ASCII, UTF-8 无 BOM) ----
    out = []
    out.append('#include "lcd_font.h"')
    out.append('#include <string.h>')
    out.append("")
    out.append("/**")
    out.append(" * @file    lcd_font_gb2312.c")
    out.append(" * @brief   GB2312 全量中文点阵 (16x16, HZK16 布局) — 由 tools/gen_font.py 生成, 勿手工编辑")
    out.append(" * @note    覆盖 %d 个有效 GB2312 字符; 按 UTF-8 字节序升序排列, 供 strcmp 二分查找" % len(items))
    out.append(" *          字模: 32 字节/字, 列式 (前16=上8行, 后16=下8行), 字节内 bit0=列顶行")
    out.append(" *          字库来源: py-zhao/mplus_hzk_12 (M+ 字体派生, 宽松开源许可)")
    out.append(" */")
    out.append("")
    out.append("const ChineseCell_t LCD_CF16x16[] = {")
    for utf8, glyph in items:
        out.append('    { "%s", {\n%s\n    } },' % (esc_utf8(utf8), glyph_bytes(glyph)))
    out.append('    { "", {0} }                                   /* 哨兵: 二分查找边界 */')
    out.append("};")
    out.append("")
    out.append("/**")
    out.append(" * @brief 二分查找汉字 16x16 字模 (字表按 UTF-8 字节序升序)")
    out.append(" * @param Utf8 UTF-8 编码的汉字字符串 (需含 '\\0' 结尾)")
    out.append(" * @return 32 字节字模指针; 未命中返回 NULL")
    out.append(" * @note   与字模数组同 TU, 可 sizeof 取完整上界, 免去手工字数宏")
    out.append(" */")
    out.append("const uint8_t *LCD_FontFind(const char *Utf8)")
    out.append("{")
    out.append("    int32_t lo = 0;")
    out.append("    int32_t hi = (int32_t)(sizeof(LCD_CF16x16) / sizeof(LCD_CF16x16[0])) - 2; /* 去掉末尾哨兵 */")
    out.append("")
    out.append("    while (lo <= hi)")
    out.append("    {")
    out.append("        int32_t mid = (lo + hi) / 2;")
    out.append("        int32_t cmp = strcmp(LCD_CF16x16[mid].Index, Utf8);")
    out.append("        if (cmp < 0)        lo = mid + 1;")
    out.append("        else if (cmp > 0)   hi = mid - 1;")
    out.append("        else                return LCD_CF16x16[mid].Data;")
    out.append("    }")
    out.append("    return NULL;")
    out.append("}")
    out.append("")

    with open(OUT_FILE, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out))
    print("已生成:", OUT_FILE)
    print("文件大小:", os.path.getsize(OUT_FILE), "字节")


if __name__ == "__main__":
    sys.exit(main())