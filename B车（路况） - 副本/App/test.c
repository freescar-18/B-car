/*!
 * @file       test.c
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
// test_ADC�����Ķ���
int16 count_test = 0,flag_test = 0,error_test = 0,error_test2 = 0; 
int16 var_test2 = 0,var_test3 = 0,var_test5 = 0,var_test6 = 0,lptmr_test = 0;
double error_sqrt_test = 0,var_sqrt_test3 = 0,var_sqrt_test5 = 0,error_test3 = 0;
// test_motor�����Ķ���    
extern uint16 ADC_GetMessage[4][SamplingNum];
extern uint16 ADC_Value[4];
extern uint16 SUM_ADC_GetMessage[4];
extern uint16 ADC_Maxing[4];
extern float ADC_Normal[4];
extern float fe,fec;
extern float speed_fec,speed_fe;
extern float steer_P;
extern int16 steerctrl;
extern float speed_forecast;
extern float speed_forecast_error;
extern int16 speedctrl_left,speedctrl_right; 
extern uint8 flag;
int16 speed_now_left,speed_now_right;


float speed_min = 2000; //���ڼ�¼��С�ٶ�
float speed_fec_max = 0; //���ڼ�¼������仯��
int8 u = 0;
//uint8 flag = 0;  //����ͣ���ı�־λ 
                 //������ֵ����С �� flag �� 1������while��flag == 1����ѭ���� 
                 //���°������ɽ� flag �� 0��������ѭ����������
                 //

//test_steering�����Ķ���
uint16 steering_test = 750;  //����ʱ���õĶ��PWM
uint32 ADC_max_test[4] = {0,0,0,0};//���������ֵ����
uint8 w;

extern float speed_power;
extern uint8 none_steerctrl; 



/*******************************************************************************
 *  @brief      test_ADC���� 
 *  @note       ADC��ʾ���Ժ���������ǰ�ڲ��Ե��ֵ����������OLED��ʾ  
                ֱ�ӷ���main��while��1)��ִ��
                ����Ϊoled��ʾ�Ĳ���

              0                     ���ŵĵ�в�ֵ
              1   ��ɫ���ֵ
              2   ��ɫ���ֵ           ��ɫ���ֵ����
              3  ����������еĲ�ֵ   �����������ֵ������Ĳ�ֵ
              4   ��ɫ���ֵ           ��ɫ���ֵ����
              5   ��ɫ���ֵ
              6
              7     �������ֵ             �ұ�����ֵ
        
 *  @warning    18/3/9  v2.0
 ******************************************************************************/
void test_ADC(void)
{       
    //LED_P6x8Str(0,0,"PTB4:"); //1
    LED_P6x8Str(0,1,"GREEN:");
    LED_P6x8Str(0,2,"BLUE:");  //3
    //LED_P6x8Str(0,3,"PTB7:");
    LED_P6x8Str(0,4,"BROWN:");  //5
    LED_P6x8Str(0,5,"ORANGE:");
      
    count_test = ftm_quad_get(FTM2);  //
    lptmr_test = lptmr_pulse_get();
    //var_test1 = adc_once(ADC1_SE10, ADC_12bit);
    var_test2 = adc_once(ADC1_SE10, ADC_12bit);
    var_test3 = adc_once(ADC1_SE13, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
    var_test5 = adc_once(ADC1_SE14, ADC_12bit);
    var_test6 = adc_once(ADC1_SE15, ADC_12bit);
    
    error_test = var_test3 - var_test5;
    error_test2 = var_test2 - var_test6;
    var_sqrt_test3 = sqrt(var_test3); //����ƽ��
    var_sqrt_test5 = sqrt(var_test5);
    error_sqrt_test = var_sqrt_test3 - var_sqrt_test5;
    //error_test3 = var_test3 / var_test5;
    
    if(count_test >= 10000)
    {
      flag_test++;
      ftm_quad_clean(FTM2);     
    }
    
    //LED_PrintShort(70,0,var_test1);
    LED_PrintShort(35,1,var_test2);
    LED_PrintShort(30,2,var_test3);
   // LED_PrintShort(33,3,var_test4);
    LED_PrintShort(35,4,var_test5);  
    LED_PrintShort(40,5,var_test6);
    LED_PrintShort(70,7,count_test);
    LED_PrintShort(0,7,lptmr_test);
    LED_PrintShort(15,3,error_test);
    LED_PrintShort(75,0,error_test2);
    //LED_PrintValueF(75,1,error_test3,1);
    LED_PrintValueF(75,3,error_sqrt_test,1);
    LED_PrintValueF(85,2,var_sqrt_test3,1);
    LED_PrintValueF(85,4,var_sqrt_test5,1);
    DELAY_MS(100);
}

/*******************************************************************************
 *  @brief      test_motor���� 
 *  @note       ֱ�ӷ���main��while��1)��ִ��
                ��Ҫ��MK60_it����Ӱ������ж�
                up ��flag = 0��
                
                �ú�����Ϊ6�� Part ����ִ���Զ�ͣ�������м򵥵ĵ�вɼ������ģ��PID������·��ʶ��
                �� up ������ͣ�������ּ���ת
               
 *  @warning    18/3/15 v4.0  v3.1�汾Ϊ�ɰ�
 ******************************************************************************/
void test_motor(void)
{
      
        speed_now_right = ftm_quad_get(FTM2);  //right��
        speed_now_left = (int)( 2.46 * (lptmr_pulse_get()) );  //4000 - 1m   20 - 1m/s  left��
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
     
        /***************  û�бջ��Ĳ��� **********************/
    /*    if(fe < 0)
        {
            speedctrl_right = (int)( speed_power * (100 * (speed_forecast - speed_forecast_error)) ); //- speed_forecast_error
            speedctrl_left = (int)( speed_power * (100 * speed_forecast) );
        }
        else
        {
            speedctrl_left = (int)( speed_power * (100 * (speed_forecast - speed_forecast_error)) );
            speedctrl_right = (int)( speed_power * (100 * speed_forecast) );
        } */
         /*****************************************************/
       
      //  if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
      //  if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
        
        /****** ��¼���õ����� *****/
        if(u<3) u++; //���ڵ�һ�μ���û����һ�ε�ֵ����˱ܿ�ǰ���ε����ݼ�¼
        else
        {
        if(speed_fec > speed_fec_max)  speed_fec_max = speed_fec; //��¼���fec
        if(speed_forecast < speed_min)  speed_min = speed_forecast; //��¼��С�ٶ�
        }
        
        /*****  Part 4 ͣ���ж�  *****/
       if((ADC_Value[0] <= 10) && (ADC_Value[1] <= 10) && (ADC_Value[2] <= 10) && (ADC_Value[3] <= 10) ) //����ĸ���ж�ƫС����flag���1��Ȼ������������ѭ��
        {                                                                                                 
       //   flag = 1;                                                                                       
        }     
     /*   if(flag == 1) // ������ѭ�������ֹͣת��
        {
         speed_forecast = 0;
          //   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0); //������ 0
         //   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0); //������ 0
       //     LED_PrintShort(50,7,speed_forecast); //��ʾ���PWM
        //  LED_PrintValueF(50,5,speed_fec_max,2); //��ʾ������仯�ʵľ���ֵ
        //     LED_PrintValueF(50,3,speed_min,2); //��ʾ��С�ٶ�
        //     DELAY_MS(100);
             
        }*/
       
        if(steerctrl <  Minsteering) steerctrl =  Minsteering;  //���ת�Ǳ���
        if(steerctrl > Maxsteering) steerctrl = Maxsteering;  //���ת�Ǳ���
        
        if(speedctrl_right < 200) speedctrl_right = 200;
        if(speedctrl_left < 200) speedctrl_left = 200;
        if(speedctrl_left > 4500) speedctrl_left = 4500;
        if(speedctrl_right > 4500) speedctrl_right = 4500;
        
        if( flag == 1 ) // ������ѭ�������ֹͣת������ʱ��Ϊ��ʮ�����ȥ�ˣ�������ôС�����Ծ�ͣ����
        {          
            speedctrl_left = 0;
            speedctrl_right = 0;   
        }        
        ftm_pwm_duty(S3010_FTM, S3010_CH,steerctrl);  //������PWM
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,speedctrl_right); //������PWM
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,speedctrl_left); //������PWM      
}

/*******************************************************************************
 *  @brief      test_steering����
 *  @note       ֱ�ӷ���main��while��1)��ִ��
                ��Ҫ��MK60_it����Ӱ������ж�
                left ��steering_test = steering_test + 5;
                right ��steering_test = steering_test - 5;
      
                ���Զ��ת�Ǻ�����oled��ʾ��������PWM,��left���Ӷ��PWM����right��С���PWM

 *  @warning    18/3/11 v4.0
 ******************************************************************************/
void test_steering(void)
{
      ftm_pwm_duty(S3010_FTM, S3010_CH,steering_test);  //���ת��
      LED_PrintShort(50,7,steering_test);  //��ʾ���PWM
}

/*******************************************************************************
 *  @brief      test_max_ADC����
 *  @note       �������вɼ�
                oled��ʾ
                

 *  @warning    18/3/19
 ******************************************************************************/
void test_max_ADC(void)
{
    ADC_GetMessage[0][1] = adc_once(ADC1_SE10, ADC_12bit); //Green
    ADC_GetMessage[1][1] = adc_once(ADC1_SE13, ADC_12bit); //blue
    ADC_GetMessage[2][1] = adc_once(ADC1_SE14, ADC_12bit); //brown
    ADC_GetMessage[3][1] = adc_once(ADC1_SE15, ADC_12bit);  //orange
    
    for(w = 0;w < 4; w++)
    {
        if( ADC_GetMessage[w][1] >= ADC_max_test[w])
            ADC_max_test[w] = ADC_GetMessage[w][1];
    }
    
    LED_PrintShort(0,0,ADC_max_test[0]);  //��ʾ��ɫ���ֵ
    LED_PrintShort(0,1,ADC_max_test[1]);  //��ʾ��ɫ���ֵ
    LED_PrintShort(0,2,ADC_max_test[2]);  //��ʾ��ɫ���ֵ
    LED_PrintShort(0,3,ADC_max_test[3]);  //��ʾ��ɫ���ֵ
            
}

/*******************************************************************************
 *  @brief      test_max_ADC_flash_write����
 *  @note       ��
                
                   
 *  @warning    18/3/19
 ******************************************************************************/
void test_max_ADC_flash_write(void)
{
    flash_erase_sector(SECTOR_NUM);                     //��������
                                                        //д��flash����ǰ����Ҫ�Ȳ�����Ӧ������(��Ȼ���ݻ���)
    flash_write(SECTOR_NUM, 0, ADC_max_test[0] );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 4, ADC_max_test[1] );   //д�����ݵ�������ƫ�Ƶ�ַΪ4������һ��д��4�ֽ�
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 8, ADC_max_test[2] ) ;  //д�����ݵ�������ƫ�Ƶ�ַΪ8������һ��д��4�ֽ�
    DELAY_MS(50);
    flash_write(SECTOR_NUM, 12, ADC_max_test[3] ) ;  //д�����ݵ�������ƫ�Ƶ�ַΪ12������һ��д��4�ֽ�
    DELAY_MS(50);
}


/*******************************************************************************
 *  @brief      test_nrf_tr  ����
 *  @note       
                

 *  @warning    18/4/7
 ******************************************************************************/
void test_nrf_tr(void)
{
  uint32 i=0;
  uint8 buff[DATA_PACKET]={0}; 
  uint8 j = 0;
  
  while(!nrf_init())
  {}
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler); //���� PORTE ���жϷ�����Ϊ PORTE_VECTORn 21 enable_irq(PORTE_IRQn); 
  enable_irq (PORTC_IRQn);
  beep_on();DELAY_MS(10);beep_off();
  
  while(1) 
  {      
    if(nrf_tx(buff,DATA_PACKET) == 1 ) //����һ�����ݰ���buff����Ϊ32�ֽڣ�  
    {//�ȴ����͹����У��˴����Լ��봦������      
        while(nrf_tx_state() == NRF_TXING); //�ȴ��������  
        if( NRF_TX_OK == nrf_tx_state () ) 
        { 
             //beep_on();DELAY();beep_off();
            
          buff[0] = j ; j++; 
          if(j>=10) j=0; 
        }
       else 
         { 
           printf("\n����ʧ��:%d",i); 
          } 
     } 
    else 
    {
      printf("\n����ʧ��:%d",i);
    }
        //DELAY();
  }
}
/*******************************************************************************
 *  @brief      test_nrf_re ����
 *  @note       
                

 *  @warning    18/4/7
 ******************************************************************************/
void test_nrf_re(void) 
{ 
   uint8 buff[DATA_PACKET]; //������ջ����� 
   uint32 relen;  

   while(!nrf_init()) //��ʼ��NRF24L01+ ,�ȴ���ʼ���ɹ�Ϊֹ 
   {  }
   //�����жϷ����� 
   set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler); //���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
   enable_irq(PORTC_IRQn);
     beep_on();DELAY_MS(10);beep_off();
 
   while(1) 
  { 
       relen = nrf_rx(buff,DATA_PACKET); //�ȴ�����һ�����ݰ������ݴ洢��buff�� 
       if(relen != 0) 
       { 
            LED_PrintShort(0,5,buff[0]);
       }
   } 
 }