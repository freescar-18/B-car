/*!
 * @file       Function.c
 * @brief      ���ຯ��
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
extern uint16 ADC_Value[4];
extern uint16 SUM_ADC_GetMessage[4];
extern uint16 ADC_Maxing[4];
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float ADC_Normal[4];
extern float fe,fec;
extern float speed_fec,speed_fe;
extern float steer_P;
extern float speed_Rule[5];
extern int16 steerctrl;
extern float speed_forecast;
extern int16 steerctrl_error;
extern int16 speed_forecast_left ;  //���ֳ���
extern int16 speed_forecast_right ;  //���ֳ���
extern int8 tab;
extern float speed_fec_max;
extern int16 speed_now_left,speed_now_right;

byte hello[][28] = {

{0x00,0xC0,0xC0,0xE0,0xB0,0x40,0xC0,0x50,0x60,0x40,0x40,0xC0,0xC0,0x40,
0x00,0x00,0x00,0x3F,0x00,0x0D,0x1E,0x17,0x17,0x16,0x3E,0x00,0x00,0x00},/*"��",0*/

{0x00,0x00,0x60,0x20,0x20,0x20,0xE0,0x60,0x20,0x20,0x20,0x00,0x00,0x00,
0x10,0x18,0x08,0x08,0x08,0x08,0x0F,0x08,0x08,0x18,0x18,0x18,0x18,0x08},/*"��",1*/

{0x00,0x80,0x80,0xB0,0xF0,0xA0,0xA0,0x60,0xE0,0x20,0x20,0xE0,0x00,0x00,
0x00,0x02,0x02,0x03,0x1F,0x1A,0x1B,0x2A,0x2B,0x3F,0x01,0x01,0x00,0x00},/*"��",2*/

{0x00,0x40,0x60,0xB0,0x50,0x60,0x40,0xF0,0xD0,0xA0,0xB0,0x80,0x00,0x00,
0x10,0x30,0x19,0x07,0x0B,0x3F,0x00,0x0F,0x1C,0x14,0x12,0x30,0x30,0x08},/*"��",3*/

{0x00,0x00,0x00,0x20,0x20,0xA0,0x70,0xB0,0x20,0x20,0x20,0x20,0x00,0x00,
0x00,0x0C,0x0C,0x04,0x07,0x05,0x05,0x3F,0x05,0x05,0x0D,0x0C,0x0C,0x04},/*"��",4*/

};

extern uint8 level;
extern uint16 cross;
extern uint16 cross_pass;

/*******************************************************************************
 *  @brief      beep_on����
 *  @note       ������һֱ��
   
                ��һ�µĺ���Ϊ
                beep_on();
                DELAY(); //��ʱ500ms
                beep_off();
 *  @warning
 ******************************************************************************/
void beep_on(void)
{
    gpio_set(PTC8,1);
}

/*******************************************************************************
 *  @brief      beep_off����
 *  @note       �������ر�
 *  @warning
 ******************************************************************************/
void beep_off(void)
{
    gpio_set(PTC8,0);
}

/*******************************************************************************
 *  @brief      oled_view����
 *  @note       oled��ʾ
 *  @warning    18/3/25
 ******************************************************************************/
void oled_view(void)
{
        LED_PrintShort(45,7,tab); 
        LED_PrintValueF(45,2,speed_forecast,2); 
     //   LED_PrintShort(90,4,speed_forecast_right); //��ʾ���PWM 
        LED_PrintValueF(0,4,speed_fec,2); //��ʾ������仯�ʵľ���ֵ
    //    LED_PrintValueF(50,3,speed_min,2); //��ʾ��С�ٶ�
    //   LED_PrintShort(0,0,ADC_Value[0]);  //��ʾ��ɫ���ֵ
   //     LED_PrintShort(0,1,ADC_Value[1]);  //��ʾ��ɫ���ֵ
   //    LED_PrintShort(0,2,ADC_Value[2]);  //��ʾ��ɫ���ֵ
   //     LED_PrintShort(0,3,ADC_Value[3]);  //��ʾ��ɫ���ֵ
       LED_PrintValueF(0,0,ADC_Normal[0],3);  //��ʾ��ɫ��й�һ����ֵ
       LED_PrintValueF(0,1,ADC_Normal[1],3);  //��ʾ��ɫ��й�һ����ֵ
       LED_PrintValueF(0,2,ADC_Normal[2],3);  //��ʾ��ɫ��й�һ����ֵ
       LED_PrintValueF(0,3,ADC_Normal[3],3);  //��ʾ��ɫ��й�һ����ֵ
       LED_PrintShort(90,0,ADC_Maxing[0]); 
       LED_PrintShort(90,1,ADC_Maxing[1]); 
       LED_PrintShort(90,2,ADC_Maxing[2]); 
       LED_PrintShort(90,3,ADC_Maxing[3]); 
        LED_PrintValueF(45,3,fe,3);  //��ʾ��������
        LED_PrintShort(45,4,level); 
    //    LED_PrintValueF(50,6,steer_P,3);  //��ʾ����� P ֵ
         LED_PrintShort(90,5,speed_now_right); 
         LED_PrintShort(0,5,speed_now_left);
      //  steerctrl_error
     //   LED_PrintValueF(25,6,fec,3); //��ʾ���仯��
         LED_PrintValueF(0,7,Rule_kp[0],3); 
          LED_PrintValueF(0,6,Rule_kp[1],3); 
     //      LED_PrintValueF(90,6,Rule_kp[3],3);  
     //     LED_PrintValueF(90,7,Rule_kp[4],3); 
          LED_PrintShort(90,6,cross);
          LED_PrintShort(90,7,cross_pass);
      //      LED_PrintValueF(90,6,Rule_kp[5],3); 
     //        LED_PrintValueF(90,7,Rule_kp[6],3); 
             
              LED_PrintValueF(45,0,Rule_kd[0],3); 
          LED_PrintValueF(45,1,Rule_kd[1],3); 
           LED_PrintValueF(45,5,Rule_kd[3],3);  
          LED_PrintValueF(45,6,Rule_kd[4],3); 
     //       LED_PrintValueF(45,5,Rule_kd[5],3);  
      //       LED_PrintValueF(45,6,Rule_kd[6],3); 
  /*   LED_PrintBMP(12,0,26,1,hello[0]);
     LED_PrintBMP(27,0,41,1,hello[1]);
     LED_PrintBMP(42,0,56,1,hello[2]);
     LED_PrintBMP(57,0,71,1,hello[3]);
     LED_PrintBMP(72,0,86,1,hello[4]); */
 // LED_P14x16Str(0,0,hello[0]);
}