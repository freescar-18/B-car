/*!
 * @file       test.h
 * @brief      ���ຯ��
 * @author     
 * @version    B��
 * @date       
 */

#ifndef __TEST_H__
#define __TEST_H__

/**************************  ����������  **************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT);
void OutPut_Data_test(void);
void OutPut_Data_test_sscom(void);

void push(uint8 chanel,uint16 data);
void sendDataToScope(void);
void Freecars_scope(void);

/****************************  �궨��  ****************************************/
//#define UartDataNum  11  //FreeCars��λ�� ����������ʾ�� ����ͨ������������λ�����øı�  ��System_Initialization.h��
#endif  //__TEST_H__  