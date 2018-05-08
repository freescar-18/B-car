/*!
 * @file       MK60_it.c
 * @brief      �жϷ�����
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
extern uint16 steering_test,motorctrl_test; //test�ļ�
extern uint16 flag;  //test�ļ�
extern uint16 jishu;  //GetMessage�ļ�
extern uint16 ADC_Maxing[4]; //���ڶ�ȡflash�д洢�������ֵ
extern uint8 adc_test; //���������ֵ�ɼ��ı�־λ
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_Rule[5];
extern uint8 huandao_flag_a,huandao_flag_b,huandao_flag_c,huandao_flag_d,huandao_flag_e,huandao_flag_f;
extern uint32 timevar = 0;
extern uint8 chaoshengbotime = 0;
extern int  length = 0;
extern uint8 flag_csb = 0;
int8 times = 0; //��ʱͣ����־λ PIT0��ʱ��
int8 ones = 0;//ֻ��дһ�����ݣ���
int8 tab = 0;


/******************************************************************************* 
 *  @brief      PIT0�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   /******  10s ͣ��  *******/
  /* if(times < 5)  //������Ϊʱ��  �� tiems < 10 ��Ϊ 10s �Զ�ͣ��
    {
      times++;
    }
    else
    {
        flag = 1;
    }
  */
    
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT1�жϷ�����
 *  @note       
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
 //  gpio_turn(PTD15); 
    test_motor(); 
 //  gpio_turn(PTD15); 
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT2�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT2);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT3�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //���жϱ�־λ
}

/*******************************************************************************
 *  @brief      PORT�Ĳο��жϷ�����
 *  @since      v5.0
 *  @warning    
 ******************************************************************************/
void PORTA_IRQHandler(void)
{
    uint8  n = 0;    //���ź�
    
    ////////////////////PTA24  DOWN ���� ///////////////////////////////////////
    n = 24;
    if(PORTA_ISFR & (1 << n))           //PTE0�����ж�
    {
        PORTA_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
        if(adc_test  == 0)
        {
            ADC_Maxing[0] = flash_read(SECTOR_NUM, 0, uint16);  //��ȡ16λ
            ADC_Maxing[1] = flash_read(SECTOR_NUM, 4, uint16);  //��ȡ16λ
            ADC_Maxing[2] = flash_read(SECTOR_NUM, 8, uint16);  //��ȡ16λ
            ADC_Maxing[3] = flash_read(SECTOR_NUM, 12, uint16);  //��ȡ16λ 2�ֽ�
        }
        adc_test = 1;
        ones = 1;
        if(tab == 0) tab = 1;//�л���
        else tab = 0;
        DELAY_MS(300);
         
        /*  ����Ϊ�û�����  */
    }
    
     ///////////////// PTA25 LEFT ����  //////////////////////////////////////// 
    n = 25;
    if(PORTA_ISFR & (1 << n))           //PTE3�����ж�
    {
        PORTA_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
       //motorctrl_test = motorctrl_test + 50;
       // steering_test = steering_test + 2;
        if(ones == 0)
        {
            flash_init();  //��ʼ��flash
            test_max_ADC_flash_write();
        }
        ones = 1;  // ֻ��дһ�Σ���
        
        if(tab == 0)
        {
        Rule_kp[0] = Rule_kp[0] - 0.1;
        Rule_kp[1] = Rule_kp[1] - 0.1;
        }
        if(tab == 1)
        {
        Rule_kd[0] = Rule_kd[0] - 0.01 *10;
        Rule_kd[1] = Rule_kd[1] - 0.01 *10;
        }
         DELAY_MS(300); 
     }
        
        /*  ����Ϊ�û�����  */
    
}
void PORTB_IRQHandler(void)
{
    uint8 n=0; 
    ////////////////////  PTB2 UP ����  ////////////////////////////////////////
    n = 2;
    if(PORTB_ISFR & (1 << n))           //PTE1�����ж�
    {
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
         flag = 0;
          ones = 1;
         jishu = 0;
         times = 0;//���ͣ��ʱ���ʱ
         huandao_flag_a = 0; huandao_flag_b = 0;//huandao_flag_c = 0; 
         huandao_flag_d = 0; huandao_flag_e = 0; //huandao_flag_f = 0;
         DELAY_MS(300);
        /*  ����Ϊ�û�����  */
    }
    /////////////  PTB3 RIGHT ����   ///////////////////////////////////////////
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTE2�����ж�
    {
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
      //  motorctrl_test = motorctrl_test - 50;
      //  steering_test = steering_test - 2;
        
        if(tab == 0)
        {
          Rule_kp[4] = Rule_kp[4] + 0.1;
          Rule_kp[3] = Rule_kp[3] + 0.1; 
        }
        if(tab == 1)
        {
          Rule_kd[4] = Rule_kd[4] + 0.01 * 10;
          Rule_kd[3] = Rule_kd[3] + 0.01 * 10; 
        } 
         DELAY_MS(300);
        /*  ����Ϊ�û�����  */
    }
    
      
    n=18;
    if(PORTB_ISFR & (1 << n))          
      {
          PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ
           // beep_on();
            
          if(gpio_get(PTB18))
          {
            beep_on();
            pit_time_start(PIT2);
            flag_csb = 1;
          }
          else
          {
            
          }          
      } 
          
}

void PORTE_IRQHandler(void)
{
    uint8  n = 0;    //���ź�     
     //PTE10 �ɻɹ�
    n = 10;
    if(PORTE_ISFR & (1 << n))           //PTE10�����ж�
    {
        PORTE_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */


        /*  ����Ϊ�û�����  */
    }

}

void PORTC_IRQHandler(void)
{
    uint8  m = 0;    //���ź�
    
    m = 10;
    if(PORTC_ISFR & (1 << m))           //PTC10�����ж�
      {
          PORTC_ISFR  = (1 << m);        //д1���жϱ�־λ
          
          nrf_handler();
      }   
}



