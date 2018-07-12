/*!
 * @file       SpeedControl.c
 * @brief      �ٶȿ��ƺ���
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"


/**************************  ȫ�ֱ���   ***************************************/
int8 m;
extern int8 pe,pec,num; 
extern float fe,fe_last,fec;
extern float eRule[5],ecRule[5];
extern float eFuzzy[2]; 
extern float ecFuzzy[2]; 
extern int16 speed_now_left,speed_now_right; 
float speed_fe = 0;  //�����������fe��ɾ���ֵ����ֵ��speed_fe
float speed_fec = 0; //�����������fec��ɾ���ֵ����ֵ��speed_fec
float speed_Fuzzy[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ������*/ 0};
float speed_forecast; //Ԥ�⽫Ҫ�ﵽ���ٶȣ�PWM��
float speed_forecast_error; //Ԥ�⽫Ҫ�ﵽ���ٶȵ�ƫ����٣�
int8 speed_pe,speed_pec;
float speed_forecast_left = 0;  //����Ԥ�⳵��
float speed_forecast_right = 0;  //����Ԥ�⳵��

/*��Ҫ���ڵĲ���*/
float speed_power = 1;
float speed_eRule[5] = {0,40,50,60,70}; //������speed_fe���ķ�Χ                                  
float speed_ecRule[5] = {0,10,20,30,40}; //�������ı仯�ʣ�speed_fec���ķ�Χ
//float speed_Rule[5] = {18,19,20,21,22}; //���Ԥ���ٶȣ�speed_forecast���ķ�Χ 
//float speed_error_Rule[5] = {7,6,5,4,0};  //Ԥ���ٶ�ƫ��ķ�Χ
                                          //
float speed_Rule[5] = {47,46,45,44,43}; //���Ԥ���ٶȣ�speed_forecast���ķ�Χ 
float speed_error_Rule[5] = {28,20,10,3,0};  //Ԥ���ٶ�ƫ��ķ�Χ
int speed_rule[6][6] =   //�ٶȹ����
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0    
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};



/****************  ���²���Ϊ�ٶȱջ����ã�����δ���������ظ����壬��ʱ���� ************/
int16 speedctrl_left,speedctrl_right; 
int16 speedctrl_error_left,speedctrl_error_right; 
int16 last_PWM_right,last_PWM_left;
float speed_fe_last_left = 0;
float speed_fe_last_right = 0;
float speed_fe_left = 0;
float speed_fe_right = 0;
float speed_fec_left = 0;
float speed_fec_right = 0;
float speed_P,speed_D;  // ����� P ֵ �� D ֵ
float speed_Fuzzy_kp[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ������*/ 0};
float speed_Fuzzy_kd[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ������*/ 0};

/*******************  ��Ҫ���ڵĲ���  ******************************************/
float speed_eRule_err[5] = {-8,-6,0,6,8}; //�������ķ�Χ                                  
float speed_ecRule_err[5] = {-5,-2,0,2,5}; //�������ı仯�ʵķ�Χ
float speed_Rule_kp[5] = {-2.5,-1,0,1,2.5};  //�����Pֵ�ķ�Χ                                      
float speed_Rule_kd[5] = {0,0,0,0,0};  //�����Dֵ�ķ�Χ
int speed_rule_kp[6][6]=  //pֵ�����
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0  
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};
int speed_rule_kd[6][6]=  //dֵ�����
{
  //ec 0 1 2 3 4  //e
      {3,1,0,0,3,5},//0    
      {2,1,1,1,2,5},//1   
      {2,1,1,1,2,5},//2    
      {3,2,2,2,3,5},//3         
      {4,3,3,3,4,5},//4
      {5,5,5,5,5,5} //
};

uint8 speed_error_power = 1;
float DDD = 0;

/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal_forecast����
 *  @note       �����ȼ��㺯��
                ����������eFuzzy[]��ecfuzzy[]���ȼ�Ϊpe��pec
 *  @warning    18/3/14 ����ο�binary-star��  v4.0
 ******************************************************************************/
void speed_fuzzy_mem_cal_forecast(void)//�����ȼ���
{
    if(fe < 0) speed_fe = -fe;  //�����������fe��ɾ���ֵ����ֵspeed_fe
    else speed_fe = fe;
    if(fec < 0) speed_fec = -fec; //�����������fe��ɾ���ֵ����ֵspeed_fec
    else speed_fec = fec;
    
    
  /*-----���������������-----*/
    if(speed_fe < speed_eRule[0])     //����ȷ��������		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      speed_pe= 0;          //?
    }
    else if(speed_fe < speed_eRule[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule[1]-speed_fe)/(speed_eRule[1]-speed_eRule[0]);
      speed_pe = 0;
    }
    else if(speed_fe < speed_eRule[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule[2] -speed_fe)/(speed_eRule[2]-speed_eRule[1]);
      speed_pe =1;
    }
    else if(speed_fe < speed_eRule[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule[3] -speed_fe)/(speed_eRule[3]-speed_eRule[2]);
      speed_pe =2;
    }
    else if(speed_fe < speed_eRule[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule[4]-speed_fe)/(speed_eRule[4]-speed_eRule[3]);
      speed_pe=3;
    }	
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      speed_pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----���仯������������-----*/
    if(speed_fec <= speed_ecRule[0])
    {
      ecFuzzy[0] =1.0;
      speed_pec = 0;
    }
    else if(speed_fec < speed_ecRule[1])
    {
      ecFuzzy[0] = (speed_ecRule[1] - speed_fec)/(speed_ecRule[1]-speed_ecRule[0]);
      speed_pec = 0 ;
    }
    else if(speed_fec < speed_ecRule[2])
    {
      ecFuzzy[0] = (speed_ecRule[2] - speed_fec)/(speed_ecRule[2]-speed_ecRule[1]);
      speed_pec = 1;
    }
    else if(speed_fec < speed_ecRule[3])
    {
      ecFuzzy[0] = (speed_ecRule[3] - speed_fec)/(speed_ecRule[3]-speed_ecRule[2]);
      speed_pec = 2 ;
    }
    else if(speed_fec < speed_ecRule[4])
    { 
      ecFuzzy[0] = (speed_ecRule[4] - speed_fec)/(speed_ecRule[4]-speed_ecRule[3]);
      speed_pec=3;
    }		
    else										
    {
      ecFuzzy[0] =1.0;
      speed_pec=4;
    }
    ecFuzzy[1] = 1.0 - ecFuzzy[0];
    
/**/speed_pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
      
}
    

/*******************************************************************************
 *  @brief      speed_fuzzy_query_forecast����
 *  @note       ��ѯģ��������������ĸ�ռ��
                
 *  @warning    18/3/14 ����ο�binary-star��  v4.0
 ******************************************************************************/
void speed_fuzzy_query_forecast(void)//��ѯģ�������
{
    for(m = 0;m <= rank; m++) //��������Ա��ۼ�
    {
        speed_Fuzzy[m] = 0;
    }
      /*��ѯkpģ�������*/  
    num = speed_rule[speed_pe][speed_pec];
    speed_Fuzzy[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule[speed_pe][speed_pec+1];
    speed_Fuzzy[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule[speed_pe+1][speed_pec];
    speed_Fuzzy[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule[speed_pe+1][speed_pec+1];
    speed_Fuzzy[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve_forecast����
 *  @note       ���ķ���ģ��
                
 *  @warning    18/3/14 ����ο�binary-star�� v4.0
 ******************************************************************************/
void speed_fuzzy_solve_forecast(void)//��ģ���õ�pdֵ
{
    speed_forecast = 0; //���P��Dֵ�Ա��ۼ�
    speed_forecast_error = 0;
    /*������ķ���ģ��*/
    for(m = 0;m < rank; m++)
    {
      speed_forecast += (speed_Fuzzy[m] * speed_Rule[m]);
      speed_forecast_error += (speed_Fuzzy[m] * speed_error_Rule[m]);
    }
   // if( speed_forecast > 55) speed_forecast = 55;
   // if( speed_forecast_error > 30) speed_forecast_error = 30;
    speed_forecast = speed_power * speed_forecast;
    speed_forecast_error = speed_error_power * speed_forecast_error;
}

/*******************************************************************************
 *  @brief      speedcontrol_forecast����
 *  @note       �ٶ�Ԥ�⺯�������Ԥ��Ҫ�ﵽ���ٶȵ�ƫ�����ƫ��仯��
                
 *  @warning    18/3/14 v4.0  δ���ԣ�����
 ******************************************************************************/
void speedcontrol_forecast(void)
{
    if(fe < 0)
        {
            speed_forecast_right = (speed_forecast - speed_forecast_error);
            speed_forecast_left = speed_forecast + speed_forecast_error / 2;
        }
        else
        {
            speed_forecast_left = (speed_forecast - speed_forecast_error);
            speed_forecast_right = speed_forecast + speed_forecast_error / 2;
        }
    
    speed_fe_last_left = speed_fe_left;
    speed_fe_left = speed_now_left - speed_forecast_left;
    speed_fec_left = speed_fe_left - speed_fe_last_left;
    
    speed_fe_last_right = speed_fe_right;
    speed_fe_right = speed_now_right - speed_forecast_right;
    speed_fec_right = speed_fe_right - speed_fe_last_right;
    
    
}



/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal����
 *  @note       �����ȼ��㺯��
                ����������eFuzzy[]��ecfuzzy[]���ȼ�Ϊpe��pec
 *  @warning    18/3/14 ����ο�binary-star��  v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_mem_cal_left(void)//�����ȼ���
{
    //////////////////////////����//////////////////////////////////
  /*-----���������������-----*/
    if(speed_fe_left < speed_eRule_err[0])     //����ȷ��������		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(speed_fe_left < speed_eRule_err[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule_err[1]-speed_fe_left) / (speed_eRule_err[1]-speed_eRule_err[0]);
      pe = 0;
    }
    else if(speed_fe_left < speed_eRule_err[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[2] -speed_fe_left) / (speed_eRule_err[2]-speed_eRule_err[1]);
      pe =1;
    }
    else if(speed_fe_left < speed_eRule_err[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[3] -speed_fe_left) / (speed_eRule_err[3]-speed_eRule_err[2]);
      pe =2;
    }
    else if(speed_fe_left < speed_eRule_err[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule_err[4]-speed_fe_left) / (speed_eRule_err[4]-speed_eRule_err[3]);
      pe=3;
    }
   
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----���仯������������-----*/
    if(speed_fec_left <= speed_ecRule_err[0])
    {
      ecFuzzy[0] =1.0;
      pec = 0;
    }
    else if(speed_fec_left < speed_ecRule_err[1])
    {
      ecFuzzy[0] = (speed_ecRule_err[1] - speed_fec_left) / (speed_ecRule_err[1]-speed_ecRule_err[0]);
      pec = 0 ;
    }
    else if(speed_fec_left < speed_ecRule_err[2])
    {
      ecFuzzy[0] = (speed_ecRule_err[2] - speed_fec_left) / (speed_ecRule_err[2]-speed_ecRule_err[1]);
      pec = 1;
    }
    else if(speed_fec_left < speed_ecRule_err[3])
    {
      ecFuzzy[0] = (speed_ecRule_err[3] - speed_fec_left) / (speed_ecRule_err[3]-speed_ecRule_err[2]);
      pec = 2 ;
    }
    else if(speed_fec_left < speed_ecRule_err[4])
    { 
      ecFuzzy[0] = (speed_ecRule_err[4] - speed_fec_left) / (speed_ecRule_err[4]-speed_ecRule_err[3]);
      pec=3;
    }   	
    else										
    {
      ecFuzzy[0] =1.0;
      pec=4;
    }
    ecFuzzy[1] = 1.0 - ecFuzzy[0];
    
/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
    
   
}

/*******************************************************************************
 *  @brief      speed_fuzzy_query����
 *  @note       ��ѯģ��������������ĸ�ռ��
                
 *  @warning    18/3/14 ����ο�binary-star��  v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_query_left(void)//��ѯģ�������
{
    for(m = 0;m <= rank; m++) //��������Ա��ۼ�
    {
        speed_Fuzzy_kp[m] = 0;
        speed_Fuzzy_kd[m] = 0;
    }
      /*��ѯkpģ�������*/  
    num = speed_rule_kp[pe][pec];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kp[pe][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kp[pe+1][pec];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kp[pe+1][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];
    
      /*��ѯkdģ�������*/  
    num = speed_rule_kd[pe][pec];
    speed_Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kd[pe][pec+1];
    speed_Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kd[pe+1][pec];
    speed_Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kd[pe+1][pec+1];
    speed_Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve����
 *  @note       ���ķ���ģ��
                
 *  @warning    18/3/14 ����ο�binary-star�� v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_solve_left(void)//��ģ���õ�pdֵ
{
    speed_P = 0; //���P��Dֵ�Ա��ۼ�
    speed_D = 0;
    /*������ķ���ģ��*/
    for(m = 0;m < rank; m++)
    {
      speed_P += speed_Fuzzy_kp[m] * speed_Rule_kp[m];
      speed_D += speed_Fuzzy_kd[m] * speed_Rule_kd[m];   
    }
}

/*******************************************************************************
 *  @brief      speedcontrol����
 *  @note       ���ת�Ǻ���
                
 *  @warning    18/3/14 v4.0 δ���ԣ�����
 ******************************************************************************/
void speedcontrol_left(void) 
{
    last_PWM_left = speedctrl_left;
    if(speed_P > 0)  speed_P = -speed_P; //������� P ֵ��Ϊ������������������ǡ��Ϊ���ת�ǵ�����
    if(speed_D > 0)  speed_D = -speed_D; //������� D ֵ��Ϊ����
    speedctrl_error_left = (int)( speed_P * speed_fe_left + 0.35 * (speed_fe_left + speed_fe_last_left) + DDD * (speed_fe_left - speed_fe_last_left) );//���ת������
    speedctrl_left = (int)(last_PWM_left + speedctrl_error_left); //���ת��PWM
}

/*******************************************************************************
 *  @brief      speed_fuzzy_mem_cal����
 *  @note       �����ȼ��㺯��
                ����������eFuzzy[]��ecfuzzy[]���ȼ�Ϊpe��pec
 *  @warning    18/3/14 ����ο�binary-star��  v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_mem_cal_right(void)//�����ȼ���
{
     //////////////////////����/////////////////////////////////////
  /*-----���������������-----*/
    if(speed_fe_right < speed_eRule_err[0])     //����ȷ��������		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(speed_fe_right < speed_eRule_err[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (speed_eRule_err[1] -speed_fe_right) / (speed_eRule_err[1]-speed_eRule_err[0]);
      pe = 0;
    }
    else if(speed_fe_right < speed_eRule_err[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[2] -speed_fe_right) / (speed_eRule_err[2]-speed_eRule_err[1]);
      pe =1;
    }
    else if(speed_fe_right < speed_eRule_err[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (speed_eRule_err[3] -speed_fe_right) / (speed_eRule_err[3]-speed_eRule_err[2]);
      pe =2;
    }
    else if(speed_fe_right < speed_eRule_err[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (speed_eRule_err[4] -speed_fe_right) / (speed_eRule_err[4]-speed_eRule_err[3]);
      pe=3;
    }
   
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----���仯������������-----*/
    if(speed_fec_right <= speed_ecRule_err[0])
    {
      ecFuzzy[0] =1.0;
      pec = 0;
    }
    else if(speed_fec_right < speed_ecRule_err[1])
    {
      ecFuzzy[0] = (speed_ecRule_err[1] - speed_fec_right) / (speed_ecRule_err[1]-speed_ecRule_err[0]);
      pec = 0 ;
    }
    else if(speed_fec_right < speed_ecRule_err[2])
    {
      ecFuzzy[0] = (speed_ecRule_err[2] - speed_fec_right) / (speed_ecRule_err[2]-speed_ecRule_err[1]);
      pec = 1;
    }
    else if(speed_fec_right < speed_ecRule_err[3])
    {
      ecFuzzy[0] = (speed_ecRule_err[3] - speed_fec_right) / (speed_ecRule_err[3]-speed_ecRule_err[2]);
      pec = 2 ;
    }
    else if(speed_fec_right < speed_ecRule_err[4])
    { 
      ecFuzzy[0] = (speed_ecRule_err[4] - speed_fec_right) / (speed_ecRule_err[4]-speed_ecRule_err[3]);
      pec=3;
    }   	
    else										
    {
      ecFuzzy[0] =1.0;
      pec=4;
    }
    ecFuzzy[1] = 1.0 - ecFuzzy[0];
    
/**/pec = 2;
/**/ecFuzzy[0] = 1;
/**/ecFuzzy[1] = 0;
}

/*******************************************************************************
 *  @brief      speed_fuzzy_query����
 *  @note       ��ѯģ��������������ĸ�ռ��
                
 *  @warning    18/3/14 ����ο�binary-star��  v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_query_right(void)//��ѯģ�������
{
    for(m = 0;m <= rank; m++) //��������Ա��ۼ�
    {
        speed_Fuzzy_kp[m] = 0;
        speed_Fuzzy_kd[m] = 0;
    }
      /*��ѯkpģ�������*/  
    num = speed_rule_kp[pe][pec];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kp[pe][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kp[pe+1][pec];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kp[pe+1][pec+1];
    speed_Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];
    
      /*��ѯkdģ�������*/  
    num = speed_rule_kd[pe][pec];
    speed_Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[0];

    num = speed_rule_kd[pe][pec+1];
    speed_Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = speed_rule_kd[pe+1][pec];
    speed_Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = speed_rule_kd[pe+1][pec+1];
    speed_Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      speed_fuzzy_solve����
 *  @note       ���ķ���ģ��
                
 *  @warning    18/3/14 ����ο�binary-star�� v4.0 δ���ԣ�����
 ******************************************************************************/
void speed_fuzzy_solve_right(void)//��ģ���õ�pdֵ
{
    speed_P = 0; //���P��Dֵ�Ա��ۼ�
    speed_D = 0;
    /*������ķ���ģ��*/
    for(m = 0;m < rank; m++)
    {
      speed_P += speed_Fuzzy_kp[m] * speed_Rule_kp[m];
      speed_D += speed_Fuzzy_kd[m] * speed_Rule_kd[m];   
    }
}

/*******************************************************************************
 *  @brief      speedcontrol����
 *  @note       ���ת�Ǻ���
                
 *  @warning    18/3/14 v4.0 δ���ԣ�����
 ******************************************************************************/
void speedcontrol_right(void) 
{
    last_PWM_right = speedctrl_right;
    if(speed_P > 0)  speed_P = -speed_P; //������� P ֵ��Ϊ������������������ǡ��Ϊ���ת�ǵ�����
    if(speed_D > 0)  speed_D = -speed_D; //������� D ֵ��Ϊ����
    speedctrl_error_right = (int)( speed_P * speed_fe_right + 0.35 * (speed_fe_right + speed_fe_last_right) + DDD * (speed_fe_right - speed_fe_last_right));//���ת������
    speedctrl_right = (int)(last_PWM_right + speedctrl_error_right); //���ת��PWM
}
    
    
