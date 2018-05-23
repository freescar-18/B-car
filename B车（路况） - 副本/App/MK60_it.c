/*!
 * @file       MK60_it.c
 * @brief      中断服务函数
 * @author     
 * @version    B车
 * @date       
 */

/**************************  包含头文件  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  全局变量   ***************************************/
int16 times = 0; //定时停车标志位 PIT0定时器
uint8 car_dis_flag = 0; //高电平开始标记位
uint16 car_dis = 0;  //超声波测距距离 单位cm
uint8 car_dis_ms = 0; //超声波测高电平的时间 单位ms
extern uint16 start_flag;
char bluetooth_data=0;//接收到的数据存在这个变量
uint8 car_dis_times = 0;
uint8 now_vol = 0;//现在的电平
uint8 last_vol = 0;//上次的电平
extern uint8 level;
extern uint16 last_stop; 
uint16 delay_flag = 0; //用于周期延时使用
extern uint16 flag;
extern uint16 dis_right;
uint8 wait_flag = 0;
extern float ADC_Normal[4];
uint8 shizi = 0;
extern uint16 dis_back;
extern float speed_power;
uint16 gameover = 0;
/******************************************************************************* 
 *  @brief      PIT0中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT0_IRQHandler(void)
{
   if(times > 0)  
    {
      times--;
    }
    else
    {
        beep_off(); 
        if(level == 1) // 正常模式
        {
            if( ADC_Normal[0] > 0.4 && ADC_Normal[3] > 0.4 )
            {
                times = 130;
                shizi++;
                beep_on();
                if( shizi == 4 )
                {    
                    uart_putchar (UART4,'3');
                    uart_putchar (UART4,'3');
                    uart_putchar (UART4,'3');
                    
                }
                if( shizi == 5 )
                {    
                    uart_putchar (UART4,'4');
                    uart_putchar (UART4,'4');
                    uart_putchar (UART4,'4');
                }                
                if( shizi == 6 )
                {
                    speed_power = 0.4; //最后一个十字减速
                }
            }
        }
    }
    
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT1中断服务函数
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
    else if( level == 100 ) //回赛道停车
    {
        gameover++;
        test_motor();
        if(gameover > 200)  flag = 1;
    }
     ///////////////////////////////////////////////////////////////////////////
    else 
    {
        test_motor();
    } 
    //test_motor();
   //gpio_turn(PTD15); 
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT2中断服务函数
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
    if(last_vol == 0 && now_vol == 1)  //有上升沿
    {
        car_dis_flag = 1;
        car_dis_ms = 0;
    }
      
    if(car_dis_flag == 1)  //超声波测试标记位
    {
        if( car_dis_ms > 150)  //超过 15ms 即为无效数据
            car_dis_flag = 0;
        else
        {
            if( gpio_get(PTC4) == 1 ) //高电平使计数增加5 即高电平时间增加0.5ms
                car_dis_ms += 10;
            else
            {   
              /*  car_dis_value[car_dis_times] = 34 * car_dis_ms;
                if(car_dis_times == 4) car_dis_times = 0;
                car_dis_times++;
              //  car_dis = 34 * car_dis_ms; //算出超声波距离  mm
                car_dis = car_dis_value[0] + car_dis_value[1] + car_dis_value[2] + car_dis_value[3] + car_dis_value[4];
                car_dis_flag = 0; //重置标记位  */
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
    PIT_Flag_Clear(PIT2);       //清中断标志位
}

/******************************************************************************* 
 *  @brief      PIT3中断服务函数
 *  @note
 *  @warning
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    
    PIT_Flag_Clear(PIT3);       //清中断标志位
}

/*!
 *  @brief      UART3测试中断服务函数
 *  @since      v5.0
 *  @warning    蓝牙通讯函数，使用必须开中断才能使用 ,在你需要发送的地方用uart_putchar函数，如果uart_putchar(UART4,'1');
 *  Sample usage:  set_vector_handler(UART3_RX_TX_VECTORn , uart3_test_handler);    //把 uart3_handler 函数添加到中断向量表，不需要我们手动调用
 */
void uart4_test_handler(void)
{

    if(uart_query(UART4) == 1)   //接收数据寄存器满
    {
      uart_getchar(UART4, &bluetooth_data);//等待接收完//*bluetooth_data = UART_D_REG(UARTN[uratn]);///////////bluetooth_data是一个char型变量，你喜欢干啥就干啥
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
        // bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
      if(bluetooth_data ==  '2') 
      {
         flag = 0;
         wait_flag = 0;
         start_flag = 300;
         level = 100;
         //bluetooth_data = 0; //////////////////这里只是为了下次蜂鸣器不响，你想干啥就干啥
      }
    }
}



