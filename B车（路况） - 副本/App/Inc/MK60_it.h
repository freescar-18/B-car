/*!
 * @file       MK60_it.h
 * @brief      �жϷ�����
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __MK60_IT_H__
#define __MK60_IT_H__


/**************************  ����������  **************************************/
void PIT0_IRQHandler(void);  //PIT0�жϷ�����
void PIT1_IRQHandler(void);  //PIT1�жϷ�����
void PIT2_IRQHandler(void);  //PIT2�жϷ�����
void PIT3_IRQHandler(void);  //PIT3�жϷ�����
void PORTE_IRQHandler(void);  //PORTE�Ĳο��жϷ�����
void PORTC_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTA_IRQHandler(void);

/****************************  �궨��  ****************************************/

#endif  //__MK60_IT_H__