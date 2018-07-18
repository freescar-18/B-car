/*!
 * @file       System_Initialization.c
 * @brief      ��ʼ������
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __SYSTEM_INITIALIZATION_H__
#define __SYSTEM_INITIALIZATION_H__

/*
 * ϵͳ��ʼ��ͷ�ļ�
 */
/**************************  ����������  **************************************/
void PIT_Initialization(void); //��ʼ�����ڶ�ʱ��
void FTM_Initialization(void); //��ʼ��FTMʱ��
void ISR_Initialization(void); //���������жϣ����Ҫ�ڳ�ʼ��֮��
void GPIO_Initialization(void); //����Io�˿ڳ�ʼ��
void ADC_Initialization(void); //��ʼ��ADCģ��
void OLED_Initialization(void); //OLED��ʼ��
void System_Initialization(void); //�ܳ�ʼ��

/****************************  �궨��  ****************************************/
#define  PIT0_TIMER  (5)  //��ʱ�� 0 ���õ�ʱ��
#define  PIT1_TIMER  (5)  //��ʱ�� 1 ���õ�ʱ��
#define  PIT2_TIMER  (1000)  //��ʱ�� 2 ���õ�ʱ��
#define  PIT3_TIMER  (4000)  //��ʱ�� 3 ���õ�ʱ�� 

#define Midsteering 672   // 768  1380
#define Minsteering 596  //right  680   1540 
#define Maxsteering 748 //left ��ת  855   1715

#define  S3010_FTM   FTM1
#define  S3010_CH    FTM_CH0 //FTM1_CH0_PIN   PTB0
#define  S3010_HZ    (50)

#define MOTOR_HZ    (13*1000)

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH4  //FTM0_CH4_PIN   PTD4   
#define MOTOR2_PWM  FTM_CH5  //FTM0_CH5_PIN   PTD5 
#define MOTOR3_PWM  FTM_CH6  //FTM0_CH6_PIN   PTD6 
#define MOTOR4_PWM  FTM_CH0  //FTM0_CH0_PIN   PTC1 

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)    //������������������ȷ����ȫ

#define UartDataNum  11  //FreeCars��λ�� ����������ʾ�� ����ͨ������������λ�����øı�

#endif  //__SYSTEM_INITIALIZATION_H__