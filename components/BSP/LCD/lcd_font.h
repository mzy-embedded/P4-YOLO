#ifndef __LCD_FONT_H
#define __LCD_FONT_H

#include <stdint.h>

/*字符集定义*/
/*以下两个宏定义只可解除其中一个的注释*/
#define LCD_CHARSET_UTF8			//定义字符集为UTF8
//#define LCD_CHARSET_GB2312		//定义字符集为GB2312

/*字模基本单元*/
typedef struct
{

#ifdef LCD_CHARSET_UTF8			//定义字符集为UTF8
	char Index[5];					//汉字索引，空间为5字节
#endif

#ifdef LCD_CHARSET_GB2312			//定义字符集为GB2312
	char Index[3];					//汉字索引，空间为3字节
#endif

	uint8_t Data[32];				//字模数据
} ChineseCell_t;

/*ASCII字模数据声明*/
extern const uint8_t LCD_F8x16[][16];
extern const uint8_t LCD_F6x8[][6];

/*汉字字模数据声明*/
extern const ChineseCell_t LCD_CF16x16[];

/*汉字字模查找 (UTF-8 索引二分查找, 字表见 lcd_font_gb2312.c, 未命中返回 NULL)*/
const uint8_t *LCD_FontFind(const char *Utf8);

/*图像数据声明*/
extern const uint8_t Diode[];
/*按照上面的格式，在这个位置加入新的图像数据声明*/
//...

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
