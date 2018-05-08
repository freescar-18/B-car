#include    "include.h"
#include    "AllFunction.h"

/**************************  �������  **************************************/
extern uint16 ADC_Value[4];
//����ģ�鷢�͵���������
int16 OutData[4]={0,0,0,0};
int16 send_b=10000;//10000��Ӧ��λ���Դ�����
/************������FreeCars2.0Э�����**************/
extern uint8 uSendBuf[UartDataNum*2]={0};                //****UartDataNum����λ������ͨ����Ŀ���豣��һ��
extern uint8 FreeCarsDataNum=UartDataNum*2;
/*
extern float speed_forecast_left;
extern float speed_forecast_right;
extern int16 speedctrl_left;
extern int16 speedctrl_right;
extern int16 speed_now_left,speed_now_right;
extern float speed_fe; 
*/
/*******************************************************************************
 *  @brief      CRC_CHECK����
 *  @note       ֱ�ӷ���main��while��1)��ִ��               
      
                �Է��͵����ݾ���crcУ�飺��������ʾ����

 *  @warning    18/3/20  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����
 ******************************************************************************/
unsigned short CRC_CHECK(unsigned char *databuf,unsigned char CRC_CNT)
{
  unsigned short CRC_Temp;
  unsigned char k,j;
  CRC_Temp = 0xffff;
  
  for(k=0;k<CRC_CNT;k++)
  {
    CRC_Temp^=databuf[k];
    for(j=0;j<8;j++)
    {if(CRC_Temp&0x01)
      CRC_Temp=(CRC_Temp>>1)^0xa001;
     else CRC_Temp=CRC_Temp>>1;
    }
  }
  return(CRC_Temp);
}

/*
2. * ����˵���� SCI ʾ�������ͺ���������һ���ֽ�
3. * ����˵����
4. OutData[] ��Ҫ���͵���ֵ���������
5. * �������أ��޷��Ž��ֵ
6. * �޸�ʱ�䣺 2013-2-10
7.  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����*/
 void OutPut_Data(void)
 {
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
 {

  temp[i] = (int)OutData[i];
  temp1[i] = (unsigned int)temp[i];

  }
  for(i=0;i<4;i++)
 {
 databuf[i*2] = (unsigned char)(temp1[i]%256);
 databuf[i*2+1] = (unsigned char)(temp1[i]/256);
 }

 CRC16 = CRC_CHECK(databuf,8);
 databuf[8] = CRC16%256;
 databuf[9] = CRC16/256;

 for(i=0;i<10;i++)
 {
 uart_putchar (UART5,(char)databuf[i]);
 }
 }

/*
2. * ����˵���� SCI ʾ�������Ժ���
3. * ����˵����
4. OutData[] ��Ҫ���͵���ֵ���������
5. * �������أ��޷��Ž��ֵ
6. * �޸�ʱ�䣺 2013-2-10
7.  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����*/
 void OutPut_Data_test(void)
 {
  OutData[0] = 12345;//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = 12345;//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = 12345;//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = 12345;//adc_once(ADC1_SE15, ADC_12bit);
  OutPut_Data();
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);
 }

/*******************************************************************************
 *  @brief      ���ڵ������ֺ���
 *  @note       ֱ�ӷ���main��while��1)��ִ��               
      
                ����Ҫ�����ݱ���ַ������ͻ���������sscom

 *  @warning   2018.3.20 4.1  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����
 ******************************************************************************/
void OutPut_Data_test_sscom(void)
{
  int16 databuff[20];
  unsigned char l;
  send_b=10000;//���������ֵ
  OutData[0] = ADC_Value[0];//adc_once(ADC1_SE10, ADC_12bit);
  OutData[1] = ADC_Value[1];//adc_once(ADC1_SE12, ADC_12bit);
    //var_test4 = adc_once(ADC1_SE13, ADC_12bit);
  OutData[2] = ADC_Value[2];//adc_once(ADC1_SE14, ADC_12bit);
  OutData[3] = ADC_Value[3];//adc_once(ADC1_SE15, ADC_12bit);
  //printf("ADC_FroBack_6[1]=%d\n",OutData[0]);
  for(l=0;l<5;l++)
  {  

   databuff[l*4] = (OutData[0]/send_b);//���ȴ����λ�����δ���
   databuff[l*4+1] = (OutData[1]/send_b);
   databuff[l*4+2] = (OutData[2]/send_b);
   databuff[l*4+3] = (OutData[3]/send_b);
   OutData[0]=OutData[0]%send_b;//�����λȥ��
   OutData[1]=OutData[1]%send_b;
   OutData[2]=OutData[2]%send_b;
   OutData[3]=OutData[3]%send_b;
   send_b=send_b/10;//����10���Ա��´εõ���λ
  }
  send_b=10000;//���¸��������ֵ
  
  for(l=0;l<20;l++) //�����ݱ�Ϊ��ӦASC��
   databuff[l] = databuff[l] + 48;
  
  
 /* databuff[1]=50;
  databuff[2]=50;
  databuff[3]=50;
 */ 
  uart_putchar (UART5,'a');//�ڴ�����������ʾ�����ݵ�ʶ��������磺����12345����������ʾa12345
   for(l=0;l<5;l++)//����OutData[]��һ������//�˶�λ���ʹ��������
   {
     uart_putchar (UART5,(char)databuff[l*4]);
    }
   uart_putchar (UART5,'b');
   for(l=0;l<5;l++)//����OutData[]�ڶ�������
   {
     uart_putchar (UART5,(char)databuff[l*4+1]);
    }
   uart_putchar (UART5,'c');
   for(l=0;l<5;l++)//����OutData[]����������
   {
     uart_putchar (UART5,(char)databuff[l*4+2]);
    }
   uart_putchar (UART5,'d');
   for(l=0;l<5;l++)//����OutData[]���ĸ�����
   {
     uart_putchar (UART5,(char)databuff[l*4+3]);
    }
}

/*******************************************************************************
 *  @brief     push()
 *  @note       ��ĳ��ͨ���������������               
      
                chanel��ͨ��     ��Ӧ��λ����ʾͨ��
                data  ������-32768~32767

 *  @warning   2018/5/08  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����
 ******************************************************************************/

void push(uint8 chanel,uint16 data)
{
    uSendBuf[chanel*2]=data/256;
    uSendBuf[chanel*2+1]=data%256;
}
/*******************************************************************************
* @brief              sendDataToScope    
* @note            ��ѯ������һ֡����, ����ʱ�������ݳ����й�
                   ����ʱ����㿴�����ĵ�

* @warning        �����Է����ж����������Ե���                
******************************************************************************/
void sendDataToScope(void)
{
  uint8 i,sum=0; 
  //ʹ����ѯ�ķ�ʽ�������ݣ�������δ���ͣ�����ͣ�ڴ˴�ֱ���������
  uart_putchar(UART4,251) ;//USendOneByte(FreeCarsUARTPort,251);
  uart_putchar(UART4,109) ;//USendOneByte(FreeCarsUARTPort,109);
  uart_putchar(UART4,37) ;//USendOneByte(FreeCarsUARTPort,37);

  sum+=(251);      //ȫ�����ݼ���У��
  sum+=(109);
  sum+=(37);
  for(i=0;i<FreeCarsDataNum;i++)
  {
     uart_putchar(UART4,uSendBuf[i]) ;    //USendOneByte(FreeCarsUARTPort,uSendBuf[i]);
    sum+=uSendBuf[i];         //ȫ�����ݼ���У��
  }
   uart_putchar(UART4,sum);// USendOneByte(FreeCarsUARTPort,sum);
}
/*******************************************************************************
 *  @brief      Freecars_scope
 *  @note       ֱ�ӷ���main��while��1)��ִ��   Freecars��λ��ʾ��������            
      
                ����Ҫ�����ݱ���ַ����͵�PC��

 *  @warning   2018/5/08  ��ֲʱ����ֲʱҪע�⵽��send_b��OutData[]���Ѿ���text.c�ļ�����
 ******************************************************************************/
void Freecars_scope(void)
{
  int i;
   //100ms����һ�����ݣ�����ͨ������ʾ������Ҳ����ʹ��delayms(100)�ķ�ʽ
     // DELAY_MS(5);
       OutData[0] = adc_once(ADC1_SE11, ADC_12bit);//ADC_Value[0];//(int)speed_forecast_left;//ADC_Value[0];//adc_once(ADC1_SE10, ADC_12bit);
       OutData[1] = adc_once(ADC1_SE12, ADC_12bit);//ADC_Value[1];//speedctrl_left;//ADC_Value[1];//;
       OutData[2] = adc_once(ADC1_SE14, ADC_12bit);//ADC_Value[2];//speed_now_left;//ADC_Value[2];//;
       OutData[3] = adc_once(ADC1_SE15, ADC_12bit);//ADC_Value[3];
       for(i=1;i<UartDataNum;i++)
       {
         push(i,OutData[i-1]);
       
       }
       sendDataToScope();		     //�ѻ�����������ݷ��͵�ʾ����
      push(0,10);         //��ȡ����һ�����ݵ�ʱ��,�ŵ�ʾ����0ͨ������������λΪ��100us���磺10��ʾ1ms��.
   // }

}