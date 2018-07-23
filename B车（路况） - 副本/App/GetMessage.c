/*!
 * @file       GetMessage.c
 * @brief      ���ݲɼ�����
 * @author     
 * @version    B��
 * @date       
 */
   
/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
extern float Rule_kd[5];
extern float Rule_kp[5];
extern float speed_pp,speed_round,speed_power;
uint16 ADC_GetMessage[5][SamplingNum]; //�ɼ������ĵ��ֵ��һ����й� SamplingNum ��
uint16 ADC_Value[5] = {0,0,0,0,0}; //�˲�ȡƽ����ĵ��ֵ
uint16 SUM_ADC_GetMessage[5] = {0,0,0,0,0}; 
uint16 ADC_Maxing[5] = {3300,3300,3300,3300,3300}; //��е����ֵ��������Ҫ�⣩
float ADC_Normal[5] = {0,0,0,0}; //��й�һ�����ֵ����Χ 0~1 ��
float fe = 0; //��������
float fe1,fe2;
float fe_last; //��һ�ε����
float fec; //���ı仯��
float error_diff_c;
float error_diff;
float error_diff_last;
extern float P_power;
extern float D_power;
uint16 i,j,k;
uint16 jishu=0;
uint16 change; //���ڽ�������ʱ���õ��н�
uint8 cross_rank = 0,line_rank = 0;
uint8 flag = 0;    //����ͣ���ı�־λ 
                  //������ֵ����С �� flag �� 1������while��flag == 1����ѭ���� 
                 //���°������ɽ� flag �� 0��������ѭ����������
                 //
uint8 huandao_flag_a=0,huandao_flag_b=0,huandao_flag_c=0,huandao_flag_d=0,huandao_flag_e=0,huandao_flag_f=0;//ʶ���뻷��
uint16 ruhuandao_jishu_a=0,ruhuandao_jishu_b=0;//�뻷��Ǽ���
uint16 chuhuandao_jishu_a=0,chuhuandao_jishu_b=0;//������Ǽ���
float ADC_Yuanhuan_L1=0.000,ADC_Yuanhuan_L2=0.000,ADC_Yuanhuan_L3=0.000,ADC_Yuanhuan_L4=0.000,ADC_Yuanhuan_L5=0.000;

extern int16 steerctrl;
extern int16 last_steerctrl;
extern int16 speed_forecast;

uint8 level = 1;
float dreams = 0.07;
uint16 cross = 0;
uint16 cross_pass = 0;
uint8 none_steerctrl = 0; 
uint16 cross_left = 0;
uint cross_right = 0;

//������ر���
uint16 round_is=0,round_in=0,round_out=0,round_over=0,round_num=0,round_stop=0,max_PWM_new=0,round_in_count=0,round_stop_flag=1;//round_vaule[3]={0},round_average[2],
//������������
/////////////////////////////////////////////////////////////////////////////// 
uint16 round_vaule=0;// round_vaule=0       ���뻷
                       // round_vaule=1       �������
                       // round_vaule=2       �����ұ�
//ʶ����ֵ
float  round_up_vaule=2.3;
float round_down_vaule=2.00;
//ɲ��ǿ��
uint8 round_stop_vaule=35,round_lr=2;

//////////////////////////////////////////////////////////////////////////////
//ʮ����ر���
uint16 cross_up=0,crossroad=0,crossroads=0;
extern uint16 max_PWM;
extern uint8 is_shizi;
extern int16 times;
/*******************************************************************************
 *  @brief      MessageProcessing����
 *  @note       ADC��Ϣ�ɼ������޹�һ�� 
                ���Ϊÿ����вɼ�SamplingNum�κ��ƽ��ֵ��ȥ����β��
 *  @warning    18/3/9 v3.1
 ******************************************************************************/
void MessageProcessing(void)
{  
    for(i = 0;i < SamplingNum; i++)//�ɼ����SamplingNum��
    {   
        //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
        ADC_GetMessage[0][i] = adc_once(ADC1_SE10, ADC_12bit); //Green
        ADC_GetMessage[1][i] = adc_once(ADC1_SE12, ADC_12bit); //blue
        //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
        ADC_GetMessage[2][i] = adc_once(ADC1_SE13, ADC_12bit); //orange
        ADC_GetMessage[3][i] = adc_once(ADC1_SE15, ADC_12bit);  //brown
        ADC_GetMessage[4][i] = adc_once(ADC1_SE11, ADC_12bit);  //new
        
    }
    
    for(i = 0;i < (SamplingNum - 1); i++)  //ð�ݷ����� ��С����
        for(j = i + 1;j < SamplingNum; j++)
            for(k = 0;k < 5; k++)  //ѡ����
            {
                if( ADC_GetMessage[k][i] >= ADC_GetMessage[k][j] )//����������
                {
                    change = ADC_GetMessage[k][i];
                    ADC_GetMessage[k][i] = ADC_GetMessage[k][j];
                    ADC_GetMessage[k][j] = change;
                }
                
            }
    
    for(i = Min_SamplingNum;i < SamplingNum - Min_SamplingNum; i++)//������
        for(k = 0;k < 5; k++)
        {
            SUM_ADC_GetMessage[k] += ADC_GetMessage[k][i];
        }
    
    for(k = 0;k < 5; k++)//ȡƽ��
    {
        ADC_Value[k] = SUM_ADC_GetMessage[k] / (SamplingNum - 2 * Min_SamplingNum);
    }
    
    for(k = 0;k < 5; k++)//�����͵����飬�Ա���һ��ʹ��
    {
        SUM_ADC_GetMessage[k] = 0;
    }
    
}

/*******************************************************************************
 *  @brief      ADCnormal����
 *  @note       ��һ����������ÿ����г��Ե�е����ֵ
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCnormal(void)
{
    for(k = 0;k < 5; k++)
    {
        ADC_Normal[k] = (float)ADC_Value[k] / (float)ADC_Maxing[k];
    }
    if(ADC_Normal[1] <= 0.001) ADC_Normal[1] = 0.001;
    if(ADC_Normal[2] <= 0.001) ADC_Normal[2] = 0.001;
    if(ADC_Normal[3] <= 0.001) ADC_Normal[3] = 0.001;
    if(ADC_Normal[0] <= 0.001) ADC_Normal[0] = 0.001;
    if(ADC_Normal[4] <= 0.001) ADC_Normal[4] = 0.001;
    
}


/*******************************************************************************
 *  @brief      ADCerror_diff����
 *  @note       ƫ�������
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void ADCerror_diff(void)
{
      fe_last = fe;  //��¼��һ�ε�ֵ  (ADC_Normal[0] * ADC_Normal[0])
      fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );
      fe2 =  sqrt(  ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
      fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);   
/**/  if(fe > 400) fe = 400; //����
/**/  if(fe < -400) fe = -400; //����
    
/**///  if( ADC_Normal[0] > 0.5 && ADC_Normal[3] > 0.5 ) level = 40;
/**///  if(level == 40) 
/**/// {
/**///      fe = -fe;
/**///  }
      fec = fe - fe_last;  //����仯��
      
   // fe = 0.65 * (ADC_Normal[2] - ADC_Normal[1]) + 0.35 * (ADC_Normal[3] - ADC_Normal[0]);
}
/*******************************************************************************
 *  @brief      Road_Id_Get����
 *  @note       ƫ�������
                
 *  @warning    18/3/9 v3.0
 ******************************************************************************/
void Road_Id_Get()
{
   /*****  Part 1 �����ж� *****/
        if( ((ADC_Normal[0] <= 0.500) || (ADC_Normal[1] <= 0.500)) && ((ADC_Normal[0] >= 0.020) || (ADC_Normal[1] >= 0.020)) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ұ�������о�ƫ�����ƫС����Ƕȴ���
        {                                                                                                        //  ����
            steerctrl = Minsteering;//last_steerctrl; //����������С������ƫ
           // beep_on(); //����������������һ��
           // DELAY_MS(20);
           // beep_off();
        }
        
        if( ((ADC_Normal[2] <= 0.500) || (ADC_Normal[3] <= 0.500)) && ((ADC_Normal[2] >= 0.020) || (ADC_Normal[3] >= 0.020)) && (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) )//������������о�ƫ���ұ�ƫС����Ƕȴ���
        {                                                                                                        //  ����
             
            steerctrl = Maxsteering;//last_steerctrl; //���������������ƫ
          //  beep_on(); //����������������һ��
          //  DELAY_MS(20);
          //  beep_off();
        }
                if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ĸ���ж�ƫС�������ǳ�ʼ״̬����flag���1��Ȼ������������ѭ��
        {                                                                                                 
          jishu++;
          steerctrl = last_steerctrl;
        }
   
         if(jishu >= 10) //��ʮ����󣬻�����ôС��flag=1���������ͣ��
        {
          flag = 1;
         //speed_forecast = 0;                  
        }
        
        if(( (ADC_Normal[0] >= 0.005) || (ADC_Normal[1] >= 0.005) || (ADC_Normal[2] >= 0.005) || (ADC_Normal[3] >= 0.005))&&(jishu < 10) ) //���ֵ�ָ�����jishu��գ�����������
        {                                                                                                 
          jishu = 0;
        }        
          
        /*****  Part 2  ·��ʶ��*****/
        
///////////////////////////////////////////���������뻷////////////////////////////////
       
        if((ADC_Normal[1] >= 0.750) && (ADC_Normal[3] >= 0.750)&&(ADC_Normal[2]>=0.600)&&((ADC_Normal[2]-ADC_Normal[0])>=0.3)&&(huandao_flag_a==0)&&(huandao_flag_c==0))
        {
          huandao_flag_a=1;         //����ʶ��㣬���������������������
          beep_on();   //DELAY_MS(30); beep_off();
        }  
              if((huandao_flag_a==1)&&(ADC_Normal[2]<=0.170)&&(huandao_flag_b==0))
              {
                 huandao_flag_b=1;  //��еĵ͹�ֵʶ���
                 beep_off();
              } 
              if((huandao_flag_b==1)&&(ADC_Normal[2]>=0.480))  //�뵺��
              {
                steerctrl = Maxsteering;
                ruhuandao_jishu_a++;
              } 
              if((ruhuandao_jishu_a>0)&&(ruhuandao_jishu_a<=150))  //�뻷���ʱ��
              {
                steerctrl = Maxsteering;
                ruhuandao_jishu_a++;
              }
              if(ruhuandao_jishu_a>150)  //�뻷�ɹ�
              {
                ruhuandao_jishu_a = 0;
                huandao_flag_a=0;
                huandao_flag_b=0;
                huandao_flag_c=1; 
              }
              if((huandao_flag_c==1)&&(ADC_Normal[0]>=0.900))  //����ʶ���
              {
                steerctrl = Maxsteering;
                speed_forecast = 2500;    //��������
                
                beep_on();
                chuhuandao_jishu_a++;
              }
              if((chuhuandao_jishu_a>0)&&(chuhuandao_jishu_a<=100)) //�������ʱ��
              {
                steerctrl = Maxsteering;
                chuhuandao_jishu_a++;
              }
              if((chuhuandao_jishu_a>100)&&(chuhuandao_jishu_a<=(100+200)))  //�������־λ    
              {
                chuhuandao_jishu_a++;
              }
              if(chuhuandao_jishu_a>100+200)
              {
                chuhuandao_jishu_a = 0;
                huandao_flag_c = 0;  
                beep_off();
              }
                            
        
 //////////////////////////////////////////����˳���뻷/////////////////////////////////////      
        
        if((ADC_Normal[1] >= 0.670) && (ADC_Normal[3] >= 0.670)&&(ADC_Normal[0]>=0.600)&&((ADC_Normal[0]-ADC_Normal[2])>=0.3)&&(huandao_flag_d==0))
        {
          huandao_flag_d=1;  //����ʶ��㣬���������������������
          beep_on();DELAY();beep_off();
        }  
              if((huandao_flag_d==1)&&(ADC_Normal[0]<=0.270))
              {
                 huandao_flag_e=1;  //��еĵ͹�ֵʶ���
                 beep_off();
              } 
              if((huandao_flag_e==1)&&(ADC_Normal[0]>=0.480))  //�뵺��
              {
                steerctrl = Minsteering;
                ruhuandao_jishu_b++;
              } 
              if((ruhuandao_jishu_b>0)&&(ruhuandao_jishu_b<=150))  //�뻷���ʱ��
              {
                steerctrl = Minsteering;
                ruhuandao_jishu_b++;
              }
              if(ruhuandao_jishu_b>150)  //�뻷�ɹ�
              {
                ruhuandao_jishu_b = 0;
                huandao_flag_f=1; 
                huandao_flag_d=0;
                huandao_flag_e=0;
              }
              if((huandao_flag_f==1)&&(ADC_Normal[2]>=0.900))  //����ʶ���
              {
                steerctrl = Minsteering;
                speed_forecast = 2500;    //��������
                beep_on();
                chuhuandao_jishu_b++;
              }
              if((chuhuandao_jishu_b>0)&&(chuhuandao_jishu_b<=100))  //�������ʱ��
              {
                steerctrl = Minsteering;
                chuhuandao_jishu_b++;
              }
              if((chuhuandao_jishu_b>100)&&(chuhuandao_jishu_b<=100+200))  //�������־λ   
              {
                chuhuandao_jishu_b++;
              }
              if(chuhuandao_jishu_b>100+200)
              {
                chuhuandao_jishu_b = 0;
                huandao_flag_f = 0; 
                beep_off();
              } 
          
     
        
        
        /*****  Part 3 ͣ���ж�  *****/
       /* if( (ADC_Normal[0] <= 0.005) && (ADC_Normal[1] <= 0.005) && (ADC_Normal[2] <= 0.005) && (ADC_Normal[3] <= 0.005) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
        {                                                                                                 
          flag = 1;                                                                                       
        }*/     
        if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
        {
         speed_forecast = 0;
         
         //steerctrl = 768;
         
           // ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������ 0
            //ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������ 0
       //     LED_PrintShort(50,7,speed_forecast); //��ʾ���PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //��ʾ������仯�ʵľ���ֵ
        //     LED_PrintValueF(50,3,speed_min,2); //��ʾ��С�ٶ�
        //     DELAY_MS(100);
             
        }
}
/*******************************************************************************
 *  @brief      road_check����
 *  @note       ·��
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void road_check(void)
{
    D_power = 1;
    if( (cross_pass > 1) || ( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ((ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5)) ) )
    {
        if( (ADC_Normal[1] > 0.75 && ADC_Normal[2] > 0.75) && ( (ADC_Normal[0] < 0.5 || ADC_Normal[3] < 0.5) ))   //�жϻ���
        {
            speed_power = 0.1;
            P_power = 2;
            cross = cross + 2;
            if( ADC_Normal[3] > ADC_Normal[0] )  
                cross_left++;
            if( ADC_Normal[0] > ADC_Normal[3] ) 
                cross_right++;
            if( (cross > 250) && (cross_pass < 50) )  
                cross_pass = cross_pass + 100;          
        }
        if( (cross_pass > 1) )
        {
            speed_power = 0.1;
            level = 31;
            if(cross > 0) cross--;
            if(cross_pass > 0) cross_pass--;
            if( cross_left > cross_right )  
            {
                fe1 =  sqrt( ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe
                fe2 =  sqrt( ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
               // none_steerctrl = 1; steerctrl =  Maxsteering;
            }//��ת
            else                           
            {
                //none_steerctrl = 1; steerctrl = Minsteering; 
            }//��ת
        }   
    } 
    else 
    {       
            if(cross > 0) cross--;
            if(cross_left > 0) cross_left--;
            if(cross_right > 0) cross_right--;
            none_steerctrl = 0;
            if(ADC_Normal[0] < 0.05 && ADC_Normal[3] < 0.05 )   // ֱ�� 
            {
                if( P_power > 0.5 ) P_power = P_power - 0.8 * dreams;   //���ͼ���Pֵ 
                fe1 =  sqrt( 2 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe
                fe2 =  sqrt( 2 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) )  * 100);
                D_power = 0.4;
                speed_power = 1.2;
                level = 1;
            }
            else
            {
               if( ( ADC_Normal[0] < 0.4 || ADC_Normal[3] < 0.4 ) ) //�������Χ��
               {
                  if( (ADC_Normal[0] > ADC_Normal[3] && ADC_Normal[2] > ADC_Normal[1] && ADC_Normal[0] > 0.05) || ( ADC_Normal[3] > ADC_Normal[0] && ADC_Normal[1] > ADC_Normal[2] && ADC_Normal[3] > 0.05 ) )  //�ж��Ƿ�����
                  {
                        fe = (int)( (sqrt( ADC_Normal[2] * ADC_Normal[2] ) - sqrt( ADC_Normal[1] * ADC_Normal[1] ) ) * 100 ); // ϵ������
                        level = 11;
                        if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                        speed_power = 0.8;
                  }
                  else
                  {      
                        if( ADC_Normal[0] < 0.05 || ADC_Normal[3] < 0.05 )  //�ж��Ƿ���һ��ֱ��ƫС
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2 )  //һ��ֱ��ƫС��һ��ֱ��ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 2; 
                            }             
                            else     //һ��ֱ��ƫС��һ��ֱ��ƫ�� 
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] + ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe *0.5��ʹ�����
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] + ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                        }
                        else 
                        {
                            if(ADC_Normal[0] < 0.2 || ADC_Normal[3] < 0.2)   //��ж�����С ����һ��ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 3;
                                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] +  ADC_Normal[3] * ADC_Normal[3] );  //���¼���fe  *0.5��ʹ�����
                                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] +  ADC_Normal[0] * ADC_Normal[0] );
                                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                            }
                            
                            else     //�������ƫ��
                            {
                                if( P_power < 2 ) P_power = P_power + dreams;   //��������Pֵ
                                speed_power = 0.8;
                                level = 2;
                            }
                        }
                  }
               }
                
            }
            if(ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )   //�ж�ʮ��
            {
                if( P_power < 1.5 ) P_power = P_power + dreams;   //��������Pֵ
                D_power = 0.6;
                speed_power = 0.5;
                fe1 =  sqrt( 0.5 * ADC_Normal[2] * ADC_Normal[2] );  //���¼���fe
                fe2 =  sqrt( 0.5 * ADC_Normal[1] * ADC_Normal[1] );
                fe = (int)(( (sqrt(fe1) - sqrt(fe2)) / ( fe1 + fe2 ) ) * 100);
                level = 21;
            }

    }
    
    if(fe > 100) fe = 100; //����
    if(fe < -100) fe = -100; //����
}


/*******************************************************************************
 *  @brief      Road_Message()����
 *  @note       ·��
                1����λ���ж� ��round_is��round_vaule��round_average��
                                round_left��round_right               
                2��ʮ���ж�  ��cross_up��cross��crossrosd
                
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void Road_Message()
{
  //�ٶȵ���
 
  if(ADC_Normal[4]<=1.6)
  {
    speed_pp=1.0;//�����ٶ�
  }
  else 
  {
    speed_pp=0.8;//�м���>1.2������
  }
   if(round_is!=0)
  {
    speed_pp=0.7;//�����ٶ�
  }
  
  //��λ���ж�      �м���ֵ>��2.00��˵���ǻ����㴦���������߽��ͳ�����ֹ���Ծ�䣬��������
  //1.6(������ֵ2700.��Сֵ1500)
  //1.95(������ֵ3400.��Сֵ1500)

  
  if((ADC_Normal[4]>=round_up_vaule)&&(round_is==0))  // round_up_vaule=2.15
  {
    round_is=1;
    if( level == 88)
      level = 4;
    else
      level = 5;
   // round_stop=10;
    //speed_pp=0.3;
  }
  else if((ADC_Normal[4]>=1.8)&&(round_is==0))///ɲ���뻷
  {
    //round_is=1;
    //round_stop_flag=1;
    if(round_stop_flag==1)
    {
      
      round_stop=round_stop_vaule;//ɲ��ǿ��///round_stop_vaule
      round_stop_flag=! round_stop_flag;
    }
    if(max_PWM_new<max_PWM)//
      max_PWM_new=max_PWM;
    max_PWM=2500;       //�ı�max_PWM����ֹɲ����˲����ٶȹ���
    
    // speed_pp=0.3;
  }
  
  if(round_is==1)//�ڻ����㴦
  {   
    if(round_vaule==2)
    {
      if(ADC_Normal[4]<=round_down_vaule)   //��Ǳ�־       round_down_vaule=1.90
    {
      round_is=2;
    //  round_stop_flag=1;
    }
    }
      
    else if(ADC_Normal[4]<=round_down_vaule)   //��Ǳ�־
    {
      round_is=2;
    //  round_stop_flag=1;
    }
    
    /*************************************************************************
    
    //�Էֱ�ֱ�����ƽ��
    round_vaule[0]+=(int)(100*ADC_Normal[0]);  //�ۼ�
    round_vaule[1]+=(int)(100*ADC_Normal[3]);
    round_vaule[2]+=1;  //�Ӻ͸���      
    
    round_average[0]=round_vaule[0]/round_vaule[2];//��ƽ��
    round_average[1]=round_vaule[1]/round_vaule[2];
    
    if(ADC_Normal[4]<=1.70)   //��Ǳ�־
    {
    //ͨ���Ƚ�ƽ��ֵ��С�ж�λ��
    //�����ұ�  round_right=1         //////////�����ʱб�壬����ֱ��й�������
    if(round_average[0]>round_average[1])
    {
    round_right=1;
    
    round_vaule[0]=round_vaule[1]=round_vaule[2]=0;
    round_average[0]=round_average[1]=0;
    
    round_is=0;
  }
    //�������   round_left=1  
    else if(round_average[0]<round_average[1])
    {
    round_left=1;
    
    round_vaule[0]=round_vaule[1]=round_vaule[2]=0;
    round_average[0]=round_average[1]=0;
    
    round_is=0;
  }      
  }
    **********************************************************************/
    
  }
  //ʮ���ж�     ����ֱ��>0.8+����ֱ��<0.1   -->����һ��ʮ�֣�crossroad=1;    ����+1��crossroads+=1��
  else if((ADC_Normal[0] >= 0.800)&&(ADC_Normal[3] >= 0.800)&&(round_is==0)&&(crossroad==0))
  {
    crossroad=1;
  }
  
  if((crossroad==1)&&(ADC_Normal[0] <= 0.750)&&(ADC_Normal[3] <= 0.750))
  {
    //crossroad=1;
    crossroad=0;
    crossroads+=1;
    
  }
  //��¼��һ�ε��ֵ
  
  //���û�����������
  Round_about();
}

/*******************************************************************************
 *  @brief      Road_about()����
 *  @note       ��������
                1���뻷�����ж� �� round_is,  round_left��round_right
                                 ���ڱ�־   round_in   

                2�����������ж�  ��round_out,   round_left��round_right
                                    ������Ǳ�־   round_over
                                  round_num
 *  @warning    18/4/7 v5.0
 ******************************************************************************/
void Round_about()
{
  if((round_vaule==3)&&(round_lr==2))
  {
    round_lr=0;
  }
  else if((round_vaule==4)&&(round_lr==2))
  {
    round_lr=1;
  }
  
  //�뻷�ж�       round_is==2
  if(round_is==2)
  {
    //�����Ҳ� 
    if((round_vaule==2)||(round_lr==1)) 
    {
      none_steerctrl=1;//�ر�ģ��pid
      steerctrl=608;   //������
      // speed_round= -13;//      ǿ�Ʋ���
      
      
      //  if((fe>20)||(fe< -20))//////ȡ����ƫ��仯��Χ
      round_in_count+=1;
      if(speed_power<0.5)
      {
        
        if(round_in_count==120)
        {
          none_steerctrl=0; //����ģ��pid
          
          round_in=1;   //
          //   speed_pp=0.2;
          round_is=3;
          //round_right=0; //
          speed_round=0; //��������
          round_in_count=0;
          
          
        }
      }
      else if(round_in_count==80)
      {
        none_steerctrl=0; //����ģ��pid
        
        round_in=1;   //
        //   speed_pp=0.2;
        round_is=3;
        //round_right=0; //
        speed_round=0; //��������
        round_in_count=0;
        
        
      }
      //round_right=0;
    }
    
    //�������
    if((round_vaule==1)||(round_lr==0))
    {
      
      none_steerctrl=1;//�ر�ģ��pid
      steerctrl=742;//������
      //speed_round=-16;//      ǿ�Ʋ���
     // speed_pp=0.5;
      round_in_count+=1;
       if(speed_power<0.5)
       {
         if(round_in_count==120)
         {
           none_steerctrl=0;
           round_in=1;
           
           
           // round_left=0;
           round_is=3;
           //speed_pp=0.1;
           speed_round=0;
           round_in_count=0;
           
         }
       }
      else if(round_in_count==80)
      {
        none_steerctrl=0;
        round_in=1;
        
      
        // round_left=0;
        round_is=3;
        //speed_pp=0.1;
        speed_round=0;
        round_in_count=0;
        
      }
      //round_left=0; 
    }
    // round_is=0
  }
  
  //��������        
  //���ڱ�־   round_in=1   
  //������־   round_over=1;
  if(round_in==1)
  {
    //
    if((ADC_Normal[0] >= 0.900)&&(ADC_Normal[3] >= 0.900))
    {
      round_out=1;//������־
    }
    
    if(round_over==1)
    {
      if(ADC_Normal[4]<=1.1)//�������־λ
      {
        round_in=0;
        round_is=0;
        round_over=0;
        round_num+=1;
        round_stop_flag=1;
        if(level == 4) //����ʮ��
            level = 88;
        else           //����ʮ��
            level = 1;
        times = 200; //����ʮ��
        is_shizi = 0; //����ʮ��
        max_PWM=max_PWM_new;//�ָ�pwm����
        if((round_lr==1)||(round_lr==0))
          round_lr=!round_lr;
        
      }
    }
    
    //��������
    if(round_out==1)
    {
      //�����Ҳ� 
      if((round_vaule==2)||(round_lr==1))
      {
        none_steerctrl=1;//�ر�ģ��pid
        steerctrl=602;   //������
        //speed_round= -13;//      ǿ�Ʋ���
        
        
        if(ADC_Normal[4]>=1.2)//////ȡ����ƫ��仯��Χ
        {
          none_steerctrl=0; //����ģ��pid
          
          round_over=1;   //������־
          //speed_pp=0.2;
          
          //round_right=0; //
          round_out=0;
          speed_round=0; //��������
          
        }
        //round_right=0;
      }
      
      //�������
      else if((round_vaule==1)||(round_lr==0))
      {
        
        none_steerctrl=1;//�ر�ģ��pid
        steerctrl=748;//������
       // speed_round=-16;//      ǿ�Ʋ���
       // speed_pp=0.1;
        
        if(ADC_Normal[4]>=1.2)
        {
          none_steerctrl=0;
          round_over=1;//������־
          
          //speed_pp=0.2;
         // round_left=0;
          round_out=0;
          speed_round=0;
        }
      }
    }
  }
}
