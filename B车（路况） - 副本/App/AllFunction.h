/*!
 * @file       AllFunction.h
 * @brief      ���к���ͷ�ļ�����(�Լ�д���ļ�)
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __ALLFUNCTION_H__
#define __ALLFUNCTION_H__

/**************************  ����ͷ�ļ�  **************************************/
#include    "MK60_it.h"
#include    "System_Initialization.h"
#include    "Function.h"
#include    "GetMessage.h"
#include    "AngleControl.h"
#include    "SpeedControl.h"
#include    "test.h"
#include    "wireless.h"

/***************************  ������־  ***************************************/
/*************************  �����µľͺ�  *************************************/
//˫�����������־                                                               
//���԰� v1.0                                                                   
//function���һЩ����                                                          
//anglecontrol�ŽǶȿ���pid                                                     
//speedcontrol���ٶȿ��ƣ�pid��                                                 
//mk60-it�Ŷ�ʱ���Ͱ������ⲿ�жϣ�                                             
//system-init�ų�ʼ��
//getmessage��·�������adc�ɼ�����
//allfunction��ͷ�ļ�����
//���Ժ󽫰������Ʒų�һ���ļ��У�

//���԰� v2.0
//1.��ÿ���ļ����������� 
//2.�����ԵĴ����ɺ���������function��
//3.��һ��һЩ�������ø���

//���԰� v3.0
//1.�����˶��ģ��pid���ο�binary�ӣ����޸��ˣ� ��anglecontrol��
//2.�����˵�вɼ�����ͨ���� ��get message��
//3.main�������ǲ���getmessage��Ĵ��룬��ɾ
//4.ÿ��������������д�������ڵı�ע
//ע�����뻹û�м���� ���ܻ��д�

//���԰� v3.1
//1.������ԭ��������getmessage��Ĵ���
//2.�����˴�����ע��
//3.������test�ļ���������ǰ�����汾ʹ�ù��Ĳ��Է���
//4.������־������ALLFunction�����鿴

//���԰� v4.0
//1.������test_steering����������ֱ��ͨ���������Զ��ת��
//2.������speed_fuzzy_mem_cal_forecast(void);
//        speed_fuzzy_query_forecast(void);
//        speed_fuzzy_solve_forecast(void);  ����������������SpeedControl����
//3.�޸���test���test_motor(void) ��������������ģ���ٶȣ���δ�ջ�
//4.ע�⣡��speedcontrol����ĺ�����δ���Եģ��ô����Ǳջ��Ĵ��룬���и��������
//  �����壬����ʱ���� ��1��warning ����ʱ����


#endif  //__ALLFUNCTION_H__