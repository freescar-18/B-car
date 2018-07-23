/*!
 * @file       button.c
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
extern uint16 ADC_Maxing[5]; //���ڶ�ȡflash�д洢�������ֵ
extern uint8 adc_test; //���������ֵ�ɼ��ı�־λ
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_Rule[5];
extern float speed_error_Rule[5];
extern uint8 huandao_flag_a,huandao_flag_b,huandao_flag_c,huandao_flag_d,huandao_flag_e,huandao_flag_f;
extern uint32 timevar = 0;
extern uint8 chaoshengbotime = 0;
extern int  length = 0;
extern uint8 flag_csb = 0;
int8 ones = 0;//ֻ��дһ�����ݣ���
int8 tab = 0;
extern int8 times; //��ʱͣ����־λ PIT0��ʱ��
extern uint16 last_stop;//�յ�ͣ����� ����1Ϊͣ��
extern uint8 car_dis_flag; //�ߵ�ƽ��ʼ���λ
extern uint16 car_dis;  //������������ ��λcm
extern uint8 car_dis_ms; //��������ߵ�ƽ��ʱ�� ��λms
extern uint16 start_flag;
extern uint8 level;
extern uint16 dis_right,dis_left;
extern uint16 speed;
extern uint16 delay_flag;
extern uint16 dis_back;
extern uint8 shizi_flag;
extern float DDD;
extern uint8 wait_flag;
extern uint8 shizi;
extern uint8 switch_mode;
extern uint8 avoid_flag_shizi;
extern uint8 last_flag_shizi;
extern float steer_D;
extern float last_speed_power;
extern uint8 go_flag_shizi;
extern uint16 max_PWM;
extern uint16 turn_car_dis;
extern uint16 last_start_flag;
extern float speed_power;
extern float eRule[5];
/////////////////////////////////////////////////////////////////////////////// 
extern uint16 round_vaule;// round_vaule=0       ���뻷
                       // round_vaule=1       �������
                       // round_vaule=2       �����ұ�
//ʶ����ֵ
extern float  round_up_vaule;
extern float round_down_vaule;
//ɲ��ǿ��
extern uint8 round_stop_vaule;
extern uint8 page_line;
extern uint8 write_flash_flag;
extern uint8 read_flash_flag;
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
        if(switch_mode == 100)//��ʼ״̬
        {
            if(adc_test  == 0)
            {
                ADC_Maxing[0] = flash_read(SECTOR_NUM, 0, uint16);  //��ȡ16λ
                ADC_Maxing[1] = flash_read(SECTOR_NUM, 4, uint16);  //��ȡ16λ
                ADC_Maxing[2] = flash_read(SECTOR_NUM, 8, uint16);  //��ȡ16λ
                ADC_Maxing[3] = flash_read(SECTOR_NUM, 12, uint16);  //��ȡ16λ 2�ֽ�
                ADC_Maxing[4] = flash_read(SECTOR_NUM, 16, uint16);  //��ȡ16λ 2�ֽ�
            }
            adc_test = 1;
            ones = 1;
            DELAY_MS(300);
/**/       // start_flag = 250;
        }
        else if(switch_mode == 0)//��ʾ��0
        {
            if(ones <= 1)
            {
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  right-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM  right-��
                DELAY_MS(3000);
                start_flag = 130;
                flag = 0;
                level = 1;
             //   level = 88;
            }
            ones = 2;
        }
        else if(switch_mode == 1)//��ʾ��1
        {
            turn_car_dis -= 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//��ʾ��2
        {
            avoid_flag_shizi--;
            DELAY_MS(300);
        }
        else if(switch_mode == 3)//��ʾ��3
        {
            last_flag_shizi--;
            DELAY_MS(300);
        }
        else if(switch_mode == 4)//��ʾ��4
        {
            if( page_line == 1)
              round_vaule--;
           else if( page_line == 2)
             round_up_vaule -= 0.03;
           else if( page_line == 3)
             round_down_vaule -= 0.03;
           else if( page_line == 4)
             round_stop_vaule -= 3;
           DELAY_MS(300);
        }
        else if(switch_mode == 5)//��ʾ��5
        {
           max_PWM -= 50;
           DELAY_MS(300);
        }
        else if(switch_mode == 6)//��ʾ��6
        {
            Rule_kp[0] = Rule_kp[0] + 0.3;
            Rule_kp[1] = Rule_kp[1] + 0.1;
            Rule_kp[3] = Rule_kp[3] - 0.1;
            Rule_kp[4] = Rule_kp[4] - 0.3;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//��ʾ��7
        {
            read_flash();
            DELAY_MS(300);
        }
        else if(switch_mode == 8)//��ʾ��8
        {
            eRule[0] = eRule[0] + 1;
            eRule[1] = eRule[1] + 1;
            eRule[3] = eRule[3] - 1;
            eRule[4] = eRule[4] - 1;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//��ʾ��9
        {
            speed_Rule[0]--;
            speed_Rule[1]--;
            speed_Rule[2]--;
            speed_Rule[3]--;
            speed_Rule[4]--;
            DELAY_MS(300);
        }
         
        /*  ����Ϊ�û�����  */
    }
    
     ///////////////// PTA25 LEFT ����  //////////////////////////////////////// 
    n = 25;
    if(PORTA_ISFR & (1 << n))           //PTE3�����ж�
    {
        PORTA_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
        if(switch_mode == 100)//��ʼ״̬
        {
            if(ones == 0)//д������
            {
                flash_init();  //��ʼ��flash
                test_max_ADC_flash_write();
            }
            ones = 1;  // ֻ��дһ�Σ���
            DELAY_MS(300);
        }
        else if(switch_mode == 0)//��ʾ��0
        {
            
        }
        else if(switch_mode == 1)//��ʾ��1
        {
           last_start_flag += 100;
           DELAY_MS(300);
        }
        else if(switch_mode == 2)//��ʾ��2
        {
           go_flag_shizi++;
           DELAY_MS(300);
        }
        else if(switch_mode == 3)//��ʾ��3
        {
           last_speed_power += 0.1;
           DELAY_MS(300);
        }
        else if(switch_mode == 4)//��ʾ��4
        {
           if( page_line != 1)
              page_line--;
            DELAY_MS(300);
        }
        else if(switch_mode == 5)//��ʾ��5
        {
           
        }
        else if(switch_mode == 6)//��ʾ��6
        {
           steer_D += 3;
           DELAY_MS(300);
        } 
        else if(switch_mode == 7)//��ʾ��7
        {
           if( write_flash_flag == 0 )
              write_flash();
           DELAY_MS(300);
        }
        else if(switch_mode == 8)//��ʾ��8
        {
            
        }
        else if(switch_mode == 9)//��ʾ��9
        {
            speed_error_Rule[0] += 2;
            speed_error_Rule[1]++;
            speed_error_Rule[2]++;
            DELAY_MS(300);
        }
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
        if(switch_mode == 100)//��ʼ״̬
        {
          
        }
        else if(switch_mode == 0)//��ʾ��0
        {
            
        }
        else if(switch_mode == 1)//��ʾ��1
        {
            turn_car_dis += 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//��ʾ��2
        {
            avoid_flag_shizi++;
            DELAY_MS(300);
        }
        else if(switch_mode == 3)//��ʾ��3
        {
            last_flag_shizi++;
            DELAY_MS(300);
        }
        else if(switch_mode == 4)//��ʾ��4
        {
            if( page_line == 1)
              round_vaule++;
           else if( page_line == 2)
             round_up_vaule += 0.03;
           else if( page_line == 3)
             round_down_vaule += 0.03;
           else if( page_line == 4)
             round_stop_vaule += 3;
           DELAY_MS(300);
        }
        else if(switch_mode == 5)//��ʾ��5
        {
            max_PWM += 50;
            DELAY_MS(300);
        }
        else if(switch_mode == 6)//��ʾ��6
        {
            Rule_kp[0] = Rule_kp[0] - 0.3;
            Rule_kp[1] = Rule_kp[1] - 0.3;
            Rule_kp[3] = Rule_kp[3] + 0.3;
            Rule_kp[4] = Rule_kp[4] + 0.3;
            DELAY_MS(300);
        } 
        else if(switch_mode == 7)//��ʾ��7
        {
           
        }
        else if(switch_mode == 8)//��ʾ��8
        {
            eRule[0] = eRule[0] - 1;
            eRule[1] = eRule[1] - 1;
            eRule[3] = eRule[3] + 1;
            eRule[4] = eRule[4] + 1;
            DELAY_MS(300);
        }
        else if(switch_mode == 9)//��ʾ��9
        {
            speed_Rule[0]++;
            speed_Rule[1]++;
            speed_Rule[2]++;
            speed_Rule[3]++;
            speed_Rule[4]++;
            DELAY_MS(300);
        }
        /*  ����Ϊ�û�����  */
    }
    /////////////  PTB3 RIGHT ����   ///////////////////////////////////////////
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTE2�����ж�
    {
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
        if(switch_mode == 100)//��ʼ״̬
        {
          
        }
        else if(switch_mode == 0)//��ʾ��0
        {
            
        }
        else if(switch_mode == 1)//��ʾ��1
        {
            last_start_flag -= 100;
            DELAY_MS(300);
        }
        else if(switch_mode == 2)//��ʾ��2
        {
           go_flag_shizi--;
           DELAY_MS(300);
        }
        else if(switch_mode == 3)//��ʾ��3
        {
           last_speed_power -= 0.1;
           DELAY_MS(300);
        }
        else if(switch_mode == 4)//��ʾ��4
        {
           if( page_line != 4)
              page_line++;
            DELAY_MS(300);
        }
        else if(switch_mode == 5)//��ʾ��5
        {
           
        }
        else if(switch_mode == 6)//��ʾ��6
        {
           steer_D -= 3;
           DELAY_MS(300);
        }  
        else if(switch_mode == 7)//��ʾ��7
        {
           
        }
        else if(switch_mode == 8)//��ʾ��8
        {
            
        }
        else if(switch_mode == 9)//��ʾ��9
        {
           speed_error_Rule[0] -= 2;
           speed_error_Rule[1]--;
           speed_error_Rule[2]--;
           DELAY_MS(300);
        }
        /*  ����Ϊ�û�����  */
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
       // beep_on();
       // last_stop = 1; //����ͣ�����/*
        if(start_flag == 0 && level != 40 && level!= 100 && level != 86)
          {
              if (level == 88) //�Լ���
              {
                  level = 40;
                  dis_back = 0;
                  dis_right = 0;
                  last_stop = 70;
                  wait_flag = 1;
                  round_vaule = 0;
              }
              else
              {
                  level = 40;
                  dis_back = turn_car_dis;
                  dis_right = 0;
                  if(speed_power < 0.5)
                  {
                      last_stop = 95;
                  }
                  else
                  {
                      last_stop = 0;
                  }
                  wait_flag = 0;
                  round_vaule = 0;
              }
             // beep_on();
          }
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
    
    ////////////////////////////������������ò���//////////////////////////////
    m = 4;
    if(PORTC_ISFR & (1 << m))          
      {
          PORTC_ISFR  = (1 << m);        //д1���жϱ�־λ
           /*  ����Ϊ�û�����  */
          //car_dis_flag = 1;
         // car_dis_ms = 0;
          /*  ����Ϊ�û�����  */
      }
    m = 2;
    if(PORTC_ISFR & (1 << m))          
      {
          PORTC_ISFR  = (1 << m);        //д1���жϱ�־λ
           /*  ����Ϊ�û�����  */ /*
          if(start_flag == 0 && level != 40 && level!= 100 && level != 86)
          {
              if (level == 88) //�Լ���
              {
                  level = 40;
                  dis_back = 0;
                  dis_right = 0;
                  last_stop = 90;
                  wait_flag = 1;
              }
              else
              {
                  level = 40;
                  dis_back = turn_car_dis;
                  dis_right = 0;
                  if(speed_power < 0.5)
                  {
                      last_stop = 80;
                  }
                  else
                  {
                      last_stop = 0;
                  }
                  wait_flag = 0;
              }
             // beep_on();
          }*/
          /*  ����Ϊ�û�����  */
      }
}
