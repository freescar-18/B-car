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
int8 times = 0; //��ʱͣ����־λ PIT0��ʱ��
uint8 car_dis_flag = 0; //�ߵ�ƽ��ʼ���λ
uint16 car_dis = 0;  //������������ ��λcm
uint8 car_dis_ms = 0; //��������ߵ�ƽ��ʱ�� ��λms
extern uint16 start_flag;
char bluetooth_data=0;//���յ������ݴ����������

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
    if(car_dis_flag == 1)  //���������Ա��λ
    {
        if( car_dis_ms > 150)  //���� 15ms ��Ϊ��Ч����
            car_dis_flag = 0;
        else
            {
            if( gpio_get(PTC4) == 1 ) //�ߵ�ƽʹ��������5 ���ߵ�ƽʱ������0.5ms
                car_dis_ms += 5;
            else
            {
              car_dis = 34 * car_dis_ms; //�������������  mm
                car_dis_flag = 0; //���ñ��λ
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
         beep_on();DELAY();beep_off();/////////////////////////��ϲ����ɶ�͸�ɶ
         bluetooth_data = 0; //////////////////����ֻ��Ϊ���´η��������죬�����ɶ�͸�ɶ
      }
    }


}



