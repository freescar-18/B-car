/*!
 * @file       GetMessage.h
 * @brief      ���ݲɼ�����
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __GETMESSAGE_H__
#define __GETMESSAGE_H__

/**************************  ����������  **************************************/
void GetMessage(void);
void MessageProcessing(void);
void ADCerror_diff(void);
void ADCnormal(void);
void Road_Id_Get(void);
void road_check(void);

/****************************  �궨��  ****************************************/
#define SamplingNum     (18)  //���ɨ��Ĵ���
#define Min_SamplingNum (3)  //�˲�ʱ��ȥ��λ�ĸ���
#define DUOJI_RUHUAN    820


#endif  //__GETMESSAGE_H__
    
