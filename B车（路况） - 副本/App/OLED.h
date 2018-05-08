#ifndef __K60_OLED_H__
#define __K60_OLED_H__

#include "include.h"
#define byte  char
#define uchar unsigned char
#define uint unsigned int
#define SCL_PIN PTA14
#define SDA_PIN PTA13
#define RST_PIN PTA15
#define DC_PIN PTA29

#define INIT 0
#define LED_IMAGE_WHITE 0   //ͼ��׵�

#define LED_SCL_Init  gpio_init(PTB9,GPO,INIT) // ʱ�ӳ�ʼ������  

#define LED_SDA_Init  gpio_init(PTB8,GPO,INIT)//���ݿ�D0   

#define LED_RST_Init  gpio_init(PTB17,GPO,INIT)//��λ���ܵ�ƽ

#define LED_DC_Init   gpio_init(PTB16,GPO,INIT)//ƫ�ó���

#define LED_SCLH  gpio_set (PTB9, 1)// ʱ�Ӷ���
#define LED_SCLL  gpio_set (PTB9, 0)

#define LED_SDAH  gpio_set (PTB8, 1)//���ݿ�D0
#define LED_SDAL  gpio_set (PTB8, 0)

#define LED_RSTH  gpio_set (PTB17, 1)//��λ���ܵ�ƽ
#define LED_RSTL  gpio_set (PTB17, 0)

#define LED_DCH   gpio_set (PTB16, 1)
#define LED_DCL   gpio_set (PTB16, 0)//ƫ�ó���

/************************************************/

void  LEDPIN_Init(void);   //LED�������ų�ʼ��

void  OLED_Init(void);

 void LED_CLS(void);

 void LED_Set_Pos(byte x, byte y);//�������꺯��
 void LED_WrDat(uint8_t data);   //д���ݺ���

 void LED_P6x8Char(byte x,byte y,byte ch);//�����ַ�
 void LED_P6x8Str(byte x,byte y,byte ch[]);//�����ַ���
 void LED_P8x16Char(byte x,byte y,byte ch);//�����ַ�
 void LED_P8x16Str(byte x,byte y,byte ch[]);//�����ַ���

void  LED_P14x16Str(byte x,byte y,byte ch[]);//���ͺ���

 void LED_PrintBMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]);//����ͼƬ

 void LED_Fill(byte dat);//����

 void LED_PrintValueC(uint8_t x, uint8_t y,char data);//�����ַ�������
 void LED_PrintValueI(uint8_t x, uint8_t y, int data);//������������
 void LED_PrintValueF(uint8_t x, uint8_t y, float data, uint8_t num);//���͸���������
void LED_PrintShort(uchar ucIdxX, uchar ucIdxY, int sData);
 void LED_Cursor(uint8_t cursor_column, uint8_t cursor_row);
 void Printf_oled();
  void prif_oled_init();
 extern char logo[];
 void DM_LCD_ShowIntS(uint16_t row,uint16_t num);
void DM_LCD_ShowSTD(void);
 void UART5_send(void );
 extern int i0;
#endif

