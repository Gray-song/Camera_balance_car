/*----------------------------------------------------------------------------------------------------
																			SSD1306


  * @file    OLED.c
  * @author  �ܳɹ���ֲ�޸�
  * @version V1.0
  * @date    2017.1.16
  * @brief   �ӱ����̴�ѧ����ʵ����
	* @pin		 SCL  D7
						 SDA	D6
						 RST  D5
             D C	D4											
  * @note    ��ֲ��Ҫ�����ţ���Ҫ���±ߵĺ궨���Լ� OLED_Init()���������� .h�ļ���OLED.h�ļ����޸�
	
				     ���ú���:
						 ������ֺ��ַ�����ַ���  EXP:  				LCD_Print(16, 32,"efg12345:",TYPE16X16,TYPE6X8);
             ��ʾ2������
             x,y :�������	 
             len :���ֵ�λ��
             size:�����С
             mode:ģʽ	0,���ģʽ;1,����ģʽ
             num:��ֵ(0~255);	   0~255		 1~3	
             EXP:        									          OLED_ShowNum(55,	0,(u8 )(NUM),len,16);	
             ��ʾС���ͼ�� ������   		            show_60x80_7725_shu((u8*)& pixel[0])  pixelΪ����ͷԭʼ�������� һά����
             ��ʾС���ͼ�� ������				          show_60x80_7725( (u8*)& pixel[0]);
             ����OLED��ʾ    			  		           	OLED_Display_On(void)
             �ر�OLED��ʾ     						          OLED_Display_Off(void)
             ����													          LCD_Fill(0x00);

----------------------------------------------------------------------------------------------------*/

#ifndef _LQOLED_H
#define _LQOLED_H

#include "MK60D10.h"      // Device header
#include "gpio.h"

//���ִ�С��Ӣ�����ִ�С
#define 	TYPE8X16		1
#define 	TYPE16X16		2
#define 	TYPE6X8			3

#define 	u8			uint8_t
#define 	u16			uint16_t
#define 	u32			uint32_t
//-----------------OLED�˿ڶ���----------------  					   



extern void OLED_Init(void);
extern void LCD_CLS(void);
extern void LCD_CLS_y(char y);
extern void LCD_CLS_line_area(u8 start_x,u8 start_y,u8 width);
extern void LCD_P6x8Str(u8 x,u8 y,u8 *ch,const u8 *F6x8);
extern void LCD_P8x16Str(u8 x,u8 y,u8 *ch,const u8 *F8x16);
extern void LCD_P14x16Str(u8 x,u8 y,u8 ch[],const u8 *F14x16_Idx,const u8 *F14x16);
extern void LCD_P16x16Str(u8 x,u8 y,u8 *ch,const u8 *F16x16_Idx,const u8 *F16x16);
//extern void LCD_Print(u8 x, u8 y, u8 *ch);
extern void LCD_PutPixel(u8 x,u8 y);
extern void LCD_Print(u8 x, u8 y, u8 *ch,u8 char_size, u8 ascii_size);
extern void LCD_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);
extern void Draw_BMP(u8 x,u8 y,const u8 *bmp); 
extern void LCD_Fill(u8 dat);
extern void show_60x80_7725(u8* DATA);
void LCD_Put_Column(u8 x,u8 y,u8 data);
extern void show_60x80_7725_shu(u8* DATA);
	void OLED_Set_Pos(unsigned char x, unsigned char y) ;
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_Display_On(void);
void OLED_Display_Off(void);
unsigned char crol(unsigned char c,unsigned char b);
void OLED_Set_Pos_t(unsigned char x, unsigned char y); 
void dis_bmp(u16 high, u16 width, u8 *p,u8 value);
#endif

