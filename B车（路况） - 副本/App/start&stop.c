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
extern float ADC_Normal[5];
extern uint16 ADC_Value[5];
extern float fe,fec;
extern int16 steerctrl;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
extern int16 speed_now_left,speed_now_right;
extern float speed_power;
extern uint8 none_steerctrl; 
extern int16 speedctrl_left,speedctrl_right; 
extern int16 speedctrl_left_opp,speedctrl_right_opp; 
extern uint16 last_stop;  //ͣ�������ã�ͣ��ʱ�������ּ�����ʽ��һ��
                          //�����Ҫ��ʱ�ٽ�����PWMΪ0���Ų�����ֳ�����
extern float speed_forecast; //Ԥ�⽫Ҫ�ﵽ���ٶȣ�PWM��
extern float speed_forecast_error; //Ԥ�⽫Ҫ�ﵽ���ٶȵ�ƫ����٣�
extern float fe,fe1,fe2,fe_last;
uint16 start_flag = 0;//�������λ��1����λΪ5ms
                      //����start_flag = 300������Ϊ1.5s���뷢������ start_car(void) )
int16 dis_left = 0;//�����ߵ��ܾ���
int16 dis_right = 0;//�����ߵ��ܾ���
extern float P_power;
extern float D_power;
uint16 dis_back = 3000;//�������˵ľ��룬���й��Ի���һ��

/*******************************************************************************
 *  @brief      start_car(void) 
 *  @note       ������ƫ
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
            
  /**/      //if(ADC_Normal[2] < 0.3  && ADC_Normal[1] < 0.05)  steerctrl = Maxsteering; //����̫С��������
  /**/      //else if(ADC_Normal[2] > 0.1 && ADC_Normal[1] > 0.6)  steerctrl = Minsteering; //�ҵ��̫�����Ҵ��
            if( ADC_Normal[2] < 0.4 &&  ADC_Normal[1] < 0.2)  steerctrl = Maxsteering;
            else steerctrl = Minsteering + 15;
            
            /////////////////////////////���///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //������PWM
            ////////////////////////////////////////////////////////////////////////
            
            ///////////////////////////�����ٶ�/////////////////////////////////////  
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,3000); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM 
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////�����ٶ�////////////////////////////////////
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,3000); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM 
            ///////////////////////////////////////////////////////////////////////
            ////////////////////////////ͣ��///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
            {                                                                                                 
           //   flag = 1;                                                                                       
            }
            if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
            {          
             //   ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  right-��
             //   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  left-��
             //   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM  left-��
             //   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM  right-��
            } 
}

/*******************************************************************************
 *  @brief      stop_car(void) 
 *  @note       ͣ��
 *  @warning    
 ******************************************************************************/
void stop_car(void)
{
            last_stop++;             
            speed_now_right = ftm_quad_get(FTM2);  //right��
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left��
	    if(speed_now_right < 0) speed_now_right = -speed_now_right;//���ֲ��㸺�ģ����ڿ��ٷ�ת
            P_power = 0.1;
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();
	  
	    /*****  Part 1 ��Ϣ�ɼ� ����ٶ�PID *****/
	    MessageProcessing(); //��Ϣ�ɼ�
    	    ADCnormal(); //�ɼ�����Ϣ��һ��
	    ADCerror_diff(); //ƫ����� ��� �� ���ı仯��
            // road_check(); 
		
	    if(none_steerctrl == 0)
	    {
		fuzzy_mem_cal(); //������� fe���� �� fec�����仯�ʣ� ��ѯ������
		fuzzy_query(); //��ѯģ�������
		fuzzy_solve(); //��ģ��
		steercontrol(); //������ƣ���� steerctrl Ϊ���ת��
	    }
			/*
	    if(gpio_get(PTE10) == 0)  
	    { 
		beep_on();
		last_stop = 1; //����ͣ�����
		speed_Rule[0] = 0;speed_Rule[1] = 0;speed_Rule[2] = 0;speed_Rule[3] = 0;speed_Rule[4] = 0;
		speed_error_Rule[0] = 0;speed_error_Rule[1] = 0;speed_error_Rule[2] = 0;speed_error_Rule[3] = 0;speed_error_Rule[4] = 0;
            }*/
				
            speed_forecast = 0;  //Ԥ���ٶ�Ϊ0
            speed_forecast_error = 0;  //Ԥ���ٶ�Ϊ0
            
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
            
            /////////////////////////////���///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //������PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////�����ٶ�/////////////////////////////////////
            if(speedctrl_left < 0)  //�����ٶ����
            {
                if(speedctrl_left < -9500) speedctrl_left_opp = 9500;
                else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9000); //������PWM  
            }
            else  //�����ٶ�û���  
            {
                if(speedctrl_left > 100) speedctrl_left = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,9000); //������PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////�����ٶ�////////////////////////////////////
            if(speedctrl_right < 0)  //�����ٶ����
            {
                if(speedctrl_right < -9500) speedctrl_right_opp = 9500;
                else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9000); //������PWM  
            }
            else  //�����ٶ�û���
            {
                if(speedctrl_right > 100) speedctrl_right = 100;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,9000); //������PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////ͣ��///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  right-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM  right-��
            } 
            //if( last_stop < 600 ) last_stop++;  //ͣ������
            if( last_stop == 100 ) { flag = 1; } //ͣ������  ����x������ǿ�Ƹ�0PWM����ֹ����
            P_power = 1;D_power = 1;
}  

/*******************************************************************************
 *  @brief      turn_car(void)  
 *  @note       ����
 *  @warning    
 ******************************************************************************/
void turn_car(void)
{
            speed_now_right = ftm_quad_get(FTM2);  //right��
	    speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left��
            ftm_quad_clean(FTM2);
	    lptmr_pulse_clean();
	  
	    /*****  Part 1 ��Ϣ�ɼ� ����ٶ�PID *****/
	    MessageProcessing(); //��Ϣ�ɼ�
    	    ADCnormal(); //�ɼ�����Ϣ��һ��
/**/        fe_last = fe;  //��¼��һ�ε�ֵ  (ADC_Normal[0] * ADC_Normal[0])
/**/        fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
/**/        fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
/**/        fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);      

/**/        fe = -fe;

/**/        fec = fe - fe_last;  //����仯��
            // road_check(); 
		
	    if(none_steerctrl == 0)
	    {
		fuzzy_mem_cal(); //������� fe���� �� fec�����仯�ʣ� ��ѯ������
		fuzzy_query(); //��ѯģ�������
		fuzzy_solve(); //��ģ��
		steercontrol(); //������ƣ���� steerctrl Ϊ���ת��
	    }
            /*
	    if(gpio_get(PTE10) == 0)  
	    { 
		beep_on();
		last_stop = 1; //����ͣ�����
		speed_Rule[0] = 0;speed_Rule[1] = 0;speed_Rule[2] = 0;speed_Rule[3] = 0;speed_Rule[4] = 0;
		speed_error_Rule[0] = 0;speed_error_Rule[1] = 0;speed_error_Rule[2] = 0;speed_error_Rule[3] = 0;speed_error_Rule[4] = 0;
            }*/
		
            ////////////////////////////////////
            flag = 0;
            dis_right += speed_now_right;
            if( dis_right <  - dis_back ) //�������˵ľ���
            {
                beep_off();
                flag = 1;
            }
            ////////////////////////////////////
         //   speed_forecast = 0;  //Ԥ���ٶ�Ϊ0
         //   speed_forecast_error = 0;  //Ԥ���ٶ�Ϊ0
            
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
            
            /////////////////////////////���///////////////////////////////////////
            if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
            if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
            ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //������PWM
            ////////////////////////////////////////////////////////////////////////           
            ///////////////////////////�����ٶ�/////////////////////////////////////
            if(1)  //�����ٶ����
            {
            //    if(speedctrl_left < -2000) speedctrl_left_opp = 2000;
            //    else speedctrl_left_opp = -speedctrl_left;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,1250); //������PWM  
            }
            else  //�����ٶ�û���  
            {
                if(speedctrl_left > 1000) speedctrl_left = 1000;
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            /////////////////////////////�����ٶ�////////////////////////////////////
            if(1)  //�����ٶ����
            {
            //    if(speedctrl_right < -8000) speedctrl_right_opp = 8000;
            //    else speedctrl_right_opp = -speedctrl_right;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1300); //������PWM  
            }
            else  //�����ٶ�û���
            {
                if(speedctrl_right > 1000) speedctrl_right = 1000;
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //������PWM  
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM 
            }
            ////////////////////////////////////////////////////////////////////////
            ////////////////////////////ͣ��///////////////////////////////////////
            if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
            {                                                                                                 
            //    flag = 1;                                                                                       
            }
            if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
            {          
                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0); //������PWM  right-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������PWM  left-��
                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0); //������PWM  right-��
            } 
           
}    



