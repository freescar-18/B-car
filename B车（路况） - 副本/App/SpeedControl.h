/*!
 * @file       SpeedControl.h
 * @brief      �ٶȿ��ƺ���
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __SPEEDCONTROL_H__
#define __SPEEDCONTROL_H__

/**************************  ����������  **************************************/
void speed_fuzzy_mem_cal_forecast(void);
void speed_fuzzy_query_forecast(void);
void speed_fuzzy_solve_forecast(void);
void speedcontrol_forecast(void);
void speed_fuzzy_mem_cal_left(void);
void speed_fuzzy_query_left(void);
void speed_fuzzy_solve_left(void);
void speedcontrol_left(void);
void speed_fuzzy_mem_cal_right(void);
void speed_fuzzy_query_right(void);
void speed_fuzzy_solve_right(void);
void speedcontrol_right(void);

/****************************  �궨��  ****************************************/


#endif  //__SPEEDCONTROL_H__