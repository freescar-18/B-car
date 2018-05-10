/*!
 * @file       System_Initialization.c
 * @brief      ��ʼ������
 * @author     
 * @version    B��
 * @date       
 */
   
/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"


/**************************  ȫ�ֱ���   ***************************************/

    
/*!
 *  @brief     ADCģ���ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void ADC_Initialization(void)
{
    //��вɼ���ʼ��
    adc_init (ADC1_SE10); //PTB4
    adc_init (ADC1_SE11); //PTB5
    adc_init (ADC1_SE12); //PTB6
    adc_init (ADC1_SE13); //PTB7
    adc_init (ADC1_SE14); //PTB10
    adc_init (ADC1_SE15); //PTB11
    //��Դ����ʼ��
  //  adc_init (ADC1_SE9); //PTB1
}

/*!
 *  @brief     PITģ���ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void PIT_Initialization(void)
{    
    //��ʼ��PIT0
    pit_init_ms(PIT0, PIT0_TIMER);                                
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler); //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    //��ʼ��PIT1
   // pit_init_ms(PIT1, PIT1_TIMER);                                
    //set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler); //����PIT1���жϷ�����Ϊ PIT1_IRQHandler
    //��ʼ��PIT2
    //pit_init_ms(PIT2, PIT2_TIMER);                                
    //set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler); //����PIT2���жϷ�����Ϊ PIT2_IRQHandler
    //��ʼ��PIT3
    pit_init_ms(PIT3, PIT3_TIMER);                                
    set_vector_handler(PIT3_VECTORn ,PIT3_IRQHandler); //����PIT3���жϷ�����Ϊ PIT3_IRQHandler
}

/*!
 *  @brief     FTMģ���ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void FTM_Initialization(void)
{
    //���������ʼ��
    //ftm_quad_init(FTM1); //FTM1 ���������ʼ��( PTA8��PIA9 )   FTM1_QDPHA_PIN
    ftm_quad_init(FTM2); //FTM2 ���������ʼ��( PTA10��PIA11 )
    //�����ʼ��
    ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ,720); //��ʼ�� ��� PWM
    //�����ʼ�� ��һ����ֻ����� MOTOR2 �� MOTOR3 ���ɶ���ת
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0); //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0); //��ʼ�� ��� PWM 
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0); //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 0); //��ʼ�� ��� PWM
    //lptmr��ʼ��-������
    lptmr_pulse_init(LPT0_ALT1,0xFFFF,LPT_Rising); 
}

/*!
 *  @brief     GPIOģ���ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0          
 *  @warning    
 *  Sample usage:       
 */
void GPIO_Initialization(void)
{
    //��������ʼ��
    gpio_init (PTA28, GPI,0);  //��ʼ�� PTB20 �ܽţ���������
    gpio_init (PTA29, GPI,0);  //��ʼ�� PTB21 �ܽţ���������
    gpio_init (PTA26, GPI,0);  //��ʼ�� PTB22 �ܽţ���������
    gpio_init (PTA27, GPI,0);  //��ʼ�� PTB23 �ܽţ���������
    //������ʼ��
    port_init(PTA24, ALT1 | IRQ_FALLING | PULLUP ); //��ʼ�� PTE0�ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    port_init(PTB2, ALT1 | IRQ_FALLING | PULLUP ); //��ʼ�� PTE1�ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    port_init(PTB3, ALT1 | IRQ_FALLING | PULLUP ); //��ʼ�� PTE2�ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    port_init(PTA25, ALT1 | IRQ_FALLING | PULLUP ); //��ʼ�� PTE3�ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    //�ɻɹܳ�ʼ��
    port_init(PTE10, ALT1 | IRQ_FALLING | PULLUP ); //��ʼ�� PTE10�ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler); //����PORTE���жϷ�����Ϊ PORTE_IRQHandler
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler); //����PORTE���жϷ�����Ϊ PORTE_IRQHandler
    set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler); //����PORTE���жϷ�����Ϊ PORTE_IRQHandler
   // set_vector_handler(PORTC_VECTORn ,PORTE_IRQHandler); //����PORTE���жϷ�����Ϊ PORTC_IRQHandler
    //��������ʼ��
    gpio_init (PTE1, GPO, 0); //��ʼ�� PTC8�ܽţ��������ʼ�͵�ƽ
    
}

/*!
 *  @brief     OLEDģ���ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void OLED_Initialization(void)
{
    //OLED��ʼ��
    OLED_Init();
}

/*!
 *  @brief     �ܳ�ʼ�� 
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void System_Initialization(void)
{
    PIT_Initialization();
    GPIO_Initialization();
    FTM_Initialization();
    ADC_Initialization(); 
    OLED_Initialization();
    uart_init (UART4, 9600);//���ڳ�ʼ��
}

/*!
 *  @brief     �ж���������
 *  @param      
 *  @param      
 *  @since      v5.0
 *  @warning    
 *  Sample usage:       
 */
void ISR_Initialization(void)
{
    
    //enable_irq(PIT0_IRQn); // ʹ��PIT0�ж�
   // enable_irq(PIT1_IRQn); // ʹ��PIT1�ж�
    //enable_irq(PIT2_IRQn); // ʹ��PIT2�ж�
   // enable_irq(PIT3_IRQn); // ʹ��PIT3�ж�
    enable_irq (PORTE_IRQn); //ʹ��PORTE�ж�
    enable_irq (PORTA_IRQn); //ʹ��PORTE�ж�
    enable_irq (PORTB_IRQn); //ʹ��PORTE�ж�
   // DisableInterrupts;
   // beep_on();  //��ʼ���ɹ���һ��
   // DELAY();
   // beep_off();
    DELAY_MS(10);
   // EnableInterrupts; //ͬʱ�����ж�
    
}
