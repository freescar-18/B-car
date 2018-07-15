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
int16 times = 0; //��ʱͣ����־λ PIT0��ʱ��
uint8 car_dis_flag = 0; //�ߵ�ƽ��ʼ���λ
uint16 car_dis = 0;  //������������ ��λcm
uint8 car_dis_ms = 0; //��������ߵ�ƽ��ʱ�� ��λms
extern uint16 start_flag;
char bluetooth_data=0;//���յ������ݴ����������
uint8 car_dis_times = 0;
uint8 now_vol = 0;//���ڵĵ�ƽ
uint8 last_vol = 0;//�ϴεĵ�ƽ
extern uint8 level;
extern uint16 last_stop; 
uint16 delay_flag = 0; //����������ʱʹ��
extern uint16 flag;
extern uint16 dis_right;
uint8 wait_flag = 0;
extern float ADC_Normal[5];
uint8 shizi = 0;
extern uint16 dis_back;
extern float speed_power;
uint8 avoid_flag_shizi = 2;
uint8 go_flag_shizi = 3;
uint8 last_flag_shizi = 4;
extern uint8 left_flag;
extern uint8 right_flag;
uint16 gameover = 0;
float last_speed_power = 0.5;
/******************************************************************************* 
 *  @brief      PIT0�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{/*
   if(times > 0)  
    {
      times--;
    }
    else
    {
        beep_off(); 
        if(level == 1) // ����ģʽ
        {
            if( ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )
            {
                times = 130;
                shizi++;
                beep_on();
                if( shizi == avoid_flag_shizi)
                {    
                    uart_putchar (UART4,'3');
                    uart_putchar (UART4,'3');
                    uart_putchar (UART4,'3');
                    speed_power = 0.5;
                    
                }
                if( shizi == go_flag_shizi)
                {    
                    uart_putchar (UART4,'4');
                    uart_putchar (UART4,'4');
                    uart_putchar (UART4,'4');
                    speed_power = 1;
                }                
                if( shizi == last_flag_shizi)
                {
                    speed_power = last_speed_power; //���һ��ʮ�ּ���
                }
            }
        }
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
   //gpio_turn(PTD15);  
    if( start_flag > 0 ) 
    {
        start_car();
        delay_flag = 30;
    }
     ///////////////////////////////////////////////////////////////////////////
    else if( level == 40 )
    {
        if(last_stop <= 100)  stop_car();
        else 
        {
            if(wait_flag == 0)
            {
                turn_car();
                if( flag == 1 )
                {
                    uart_putchar (UART4,'1');
                    uart_putchar (UART4,'1');
                    wait_flag = 1;
                }
            }
            else
            {
                //start_flag = 800;
                last_stop = 0; 
                dis_right = 0;
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////////
    else if( level == 100 ) //������ͣ��
    {
        gameover++;
        test_motor();
        if(gameover > 250)  flag = 1;
    }
     ///////////////////////////////////////////////////////////////////////////
    else 
    {
       test_motor();
    } 
    //test_motor();
   //gpio_turn(PTD15); 
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/******************************************************************************* 
 *  @brief      PIT2�жϷ�����
 *  @note
 *  @warning
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
    last_vol = now_vol;
    if( gpio_get(PTC4) == 1 )
        now_vol = 1;
    else 
        now_vol = 0;
    if(last_vol == 0 && now_vol == 1)  //��������
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //���������Ա��λ
    {
        if( car_dis_ms > 150)  //���� 15ms ��Ϊ��Ч����
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //�ߵ�ƽʹ��������5 ���ߵ�ƽʱ������0.5ms
                car_dis_ms += 10;
            else
            {   
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //�������������  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //���ñ��λ  */
                if( car_dis < 1500 )   
		{
                   // if( start_flag < 5) start_flag = 10;
                    //else if(start_flag < 600) start_flag = start_flag * 3;
                   // start_flag = 600;
                    //beep_on();
                    level = 40;
		}
                if( car_dis > 1500 )   
		{
                   // if( start_flag < 5) start_flag = 10;
                    //else if(start_flag < 600) start_flag = start_flag * 3;
                   // start_flag = 600;
                    //beep_off();
		}
                car_dis_flag = 0;
                car_dis = 34 * car_dis_ms; 
            }
        }        
    } 
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

/*!
 *  @brief      UART3�����жϷ�����
 *  @since      v5.0
 *  @warning    ����ͨѶ������ʹ�ñ��뿪�жϲ���ʹ�� ,������Ҫ���͵ĵط���uart_putchar���������uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //�� uart3_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void uart4_test_handler(void)
{

    if(uart_query(UART4) == 1)   //�������ݼĴ�����
    {
      uart_getchar(UART4, &bluetooth_data);//�ȴ�������//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data��һ��char�ͱ�������ϲ����ɶ�͸�ɶ
      if(bluetooth_data ==  '1') 
      {
         if( wait_flag == 1 )
         {
              uart_putchar (UART4,'2'); 
              uart_putchar (UART4,'2'); 
              flag = 0;
              wait_flag = 0;
              start_flag = 300;
              level = 100;
         }
        // bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
      if(bluetooth_data ==  '2') 
      {
         flag = 0;
         wait_flag = 0;
         start_flag = 300;
         level = 100;
         //bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
    }
}



