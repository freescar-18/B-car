/*!
 * @file       start&stop.c
 * @brief      ��Ÿ���֮ǰ���Եĺ���
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
extern float ADC_Normal[4];
extern uint16 ADC_Value[4];
extern float fe,fec;
extern int16 steerctrl;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
extern int16 speed_now_left,speed_now_right;
extern float speed_power;
extern uint8 none_steerctrl; 
extern int16 speedctrl_left,speedctrl_right; 
extern int16 speedctrl_left_opp,speedctrl_right_opp; 
extern uint8 last_stop;  //�յ�ͣ�����λ  0Ϊ����ͣ��
extern float speed_forecast; //Ԥ�⽫Ҫ�ﵽ���ٶȣ�PWM��
extern float speed_forecast_error; //Ԥ�⽫Ҫ�ﵽ���ٶȵ�ƫ����٣�
extern float fe,fe1,fe2,fe_last;
uint16 start_flag = 0;

/*******************************************************************************
 *  @brief      start_car(void) 
 *  @note       
 *  @warning    
 ******************************************************************************/
void start_car(void)
{           
            start_flag--;
            speed_now_right = ftm_quad_get(FTM2);  //right��
            speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left��
            ftm_quad_clean(FTM2);
            lptmr_pulse_clean();
      
            /*****  Part 1 ��Ϣ�ɼ� ����ٶ�PID *****/
            MessageProcessing(); //��Ϣ�ɼ�
            ADCnormal(); //�ɼ�����Ϣ��һ��
        
            
 /**/       fe_last = fe;  //��¼��һ�ε�ֵ  (ADC_Normal[0] * ADC_Normal[0])
 /**/       fe1 =  sqrt(  ADC_Normal[3] * ADC_Normal[3] );
 /**/       fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
 /**/       fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
 /**/       fec = fe - fe_last;  //����仯��      
           
 
         // road_check();
            fuzzy_mem_cal(); //������� fe���� �� fec�����仯�ʣ� ��ѯ������
            fuzzy_query(); //��ѯģ�������
            fuzzy_solve(); //��ģ��
            steercontrol(); //������ƣ���� steerctrl Ϊ���ת��
            speed_fuzzy_mem_cal_forecast(); //������� fe�����ľ���ֵ �� fec�����仯�ʣ��ľ���ֵ ��ѯ������
            speed_fuzzy_query_forecast(); //��ѯģ��������ٶȣ�
            speed_fuzzy_solve_forecast(); //��ģ�� ��� speed_forecast ΪԤ�ڵ��ٶ�
            speedcontrol_forecast();
            speed_fuzzy_mem_cal_left();
            speed_fuzzy_query_left();
            speed_fuzzy_solve_left();
            speedcontrol_left();
            speed_fuzzy_mem_cal_right();
            speed_fuzzy_query_right();
            speed_fuzzy_solve_right();
            speedcontrol_right(); 
         // Road_Id_Get();
            
  /**/      if(ADC_Normal[2] < 0.3  && ADC_Normal[1] < 0.05)  steerctrl = Maxsteering; //����̫С��������
  /**/      else if(ADC_Normal[2] > 0.1 && ADC_Normal[1] > 0.6)  steerctrl = Minsteering; //�ҵ��̫�����Ҵ��

            
            /////////////////////////////���///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //������PWM
            ////////////////////////////////////////////////////////////////////////
            
            ///////////////////////////�����ٶ�/////////////////////////////////////
            if(0)  //�����ٶ����  
            {
                if(speedctrl_left < -200) speedctrl_left_opp = 200;
                else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,speedctrl_left_opp); //������PWM  
            }
            else  //�����ٶ�û���
            {   
                if(speedctrl_left < 1500) speedctrl_left = 1500;
                if(speedctrl_left > 1500) speedctrl_left = 1500;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////�����ٶ�////////////////////////////////////
            if(0)  //�����ٶ����
            {
                if(speedctrl_right < -200) speedctrl_right_opp = 200;
                else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,speedctrl_right_opp); //������PWM  
            }
            else  //�����ٶ�û���
            {
                if(speedctrl_right < 1500) speedctrl_right = 1500;
                if(speedctrl_right > 1500) speedctrl_right = 1500;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM 
            }
            ///////////////////////////////////////////////////////////////////////
            ////////////////////////////ͣ��///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
            {                                                                                                 
           //   flag = 1;                                                                                       
            }
            if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  right-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM  right-��
            } 
}



