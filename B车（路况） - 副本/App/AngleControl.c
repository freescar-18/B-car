/*!
 * @file       AngleControl.c
 * @brief      �Ƕȿ��ƺ���
 * @author     
 * @version    B��
 * @date       
 */

/**************************  ����ͷ�ļ�  **************************************/
//#include    "common.h"
#include    "include.h"
#include    "AllFunction.h"

/**************************  ȫ�ֱ���   ***************************************/
int8 l;
int8 pe,pec,num; 
int16 steerctrl; // ����Ķ��ת��PWM
int16 last_steerctrl; // �ϴ�����Ķ��ת��PWM
int16 steerctrl_error; //���ת�ǵ������������ɸ��� ���϶���м�ֵ��Ϊ�����Ҫת�ǵ�PWM

float steer_P,steer_D;  // ����� P ֵ �� D ֵ
extern float fe,fec,fe_last; 
float eFuzzy[2] = {0,0}; 
float ecFuzzy[2] = {0,0}; 
float Fuzzy_kp[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ������*/ 0};
float Fuzzy_kd[6] = {0,0,0,0,0,/*ĩβΪ0,������ѯģ������*/ 0};

/*��Ҫ���ڵĲ���*/
float P_power = 1;
float D_power = 1;
float eRule[5] = {-50,-25,0,25,50}; //������fe���ķ�Χ���ɸ����� 
                                            //���һ��ƫ�����������Ϊ-0.9 �� 0.9������100��Ϊ-90��90���ٷֳ�7��
float ecRule[5] = {-25,-15,0,15,25}; //�������ı仯�ʣ�fec���ķ�Χ���ɸ�����
float Rule_kp[5] = {-0.7,-0.35,0,0.4,0.8};  //  �����Pֵ�ķ�Χ  
                                                   //  ��������ת��Pֵ��left -- right��(+85 - -75)
float Rule_kd[5] = {-5,-2.5,0,2.5,5};  //�����Dֵ�ķ�Χ
/*
int rule_kp[8][8]=  //pֵ�����
{
  //ec 0 1 2 3 4 5 6  //e
      {6,6,5,5,4,3,3,7},//0    
      {6,6,5,4,4,3,2,7},//1   
      {6,5,5,4,3,2,2,7},//2    
      {5,5,4,3,2,1,1,7},//3         
      {4,4,3,2,2,1,0,7},//4
      {4,3,2,1,1,1,0,7},//5
      {3,3,2,2,2,0,0,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
int rule_kp[6][6]=  //pֵ�����
{
  //ec 0 1 2 3 4  //e
      {4,4,4,3,2,5},//0    
      {4,3,3,2,1,5},//1   
      {4,3,2,1,0,5},//2    
      {3,2,1,1,0,5},//3         
      {2,1,0,0,0,5},//4
      {5,5,5,5,5,5} //
};
/*
int rule_kd[8][8]=  //dֵ�����
{
  //ec 0 1 2 3 4 5 6  //e
      {0,0,1,3,5,6,6,7},//0    
      {0,1,2,3,4,5,6,7},//1   
      {1,2,2,3,4,4,5,7},//2   
      {3,3,3,3,3,3,3,7},//3    
      {5,4,4,3,2,2,1,7},//4
      {6,5,4,3,2,1,0,7},//5
      {6,6,5,3,1,0,0,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
/*
int rule_kd[8][8]=  //dֵ�����
{
  //ec 0 1 2 3 4 5 6  //e
      {6,6,5,3,1,0,0,7},//0    
      {6,5,4,3,2,1,0,7},//1   
      {5,4,4,3,2,2,1,7},//2   
      {3,3,3,3,3,3,3,7},//3    
      {1,2,2,3,4,4,5,7},//4
      {0,1,2,3,4,5,6,7},//5
      {0,0,1,3,5,6,6,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
/*
int rule_kd[8][8]=  //dֵ�����
{
  //ec 0 1 2 3 4 5 6  //e
      {0,0,1,2,2,3,3,7},//0    
      {0,1,1,1,2,3,4,7},//1   
      {0,1,2,2,3,4,4,7},//2   
      {1,2,2,3,4,4,5,7},//3    
      {2,2,3,4,4,5,6,7},//4
      {2,3,4,5,5,5,6,7},//5
      {3,3,4,4,5,6,6,7},//6
      {7,7,7,7,7,7,7,7} //
};*/
int rule_kd[6][6]=  //pֵ�����
{
  //ec 0 1 2 3 4  //e
      {3,1,0,0,3,5},//0    
      {2,1,1,1,2,5},//1   
      {2,1,1,1,2,5},//2    
      {3,2,2,2,3,5},//3         
      {4,3,3,3,4,5},//4
      {5,5,5,5,5,5} //
};




/*******************************************************************************
 *  @brief      fuzzy_mem_cal����
 *  @note       �����ȼ��㺯��
                ����������eFuzzy[]��ecfuzzy[]���ȼ�Ϊpe��pec
 *  @warning    18/3/10 ����ο�binary-star��  v3.0
 ******************************************************************************/
void fuzzy_mem_cal(void)//�����ȼ���
{
  /*-----���������������-----*/
    if(fe < eRule[0])     //����ȷ��������		        // |x_x_x_x_x_x_x_
    {
      eFuzzy[0] =1.0; 
      pe= 0;          //?
    }
    else if(fe < eRule[1])	        // _x|x_x_x_x_x_x_
    {       
      eFuzzy[0] = (eRule[1]-fe)/(eRule[1]-eRule[0]);
      pe = 0;
    }
    else if(fe < eRule[2])	        // _x_x|x_x_x_x_x_
    {
      eFuzzy[0] = (eRule[2] -fe)/(eRule[2]-eRule[1]);
      pe =1;
    }
    else if(fe < eRule[3])	        // _x_x_x|x_x_x_x_
    {
      eFuzzy[0] = (eRule[3] -fe)/(eRule[3]-eRule[2]);
      pe =2;
    }
    else if(fe < eRule[4])		        // _x_x_x_x|x_x_x_
    {   
      eFuzzy[0] = (eRule[4]-fe)/(eRule[4]-eRule[3]);
      pe=3;
    }	
    else						        // _x_x_x_x_x_x_x|
    {
      eFuzzy[0] =1.0;
      pe=4;
    }
    eFuzzy[1] = 1.0 - eFuzzy[0];                    //eFuzzy[0]+eFuzzy[1]=1;
    
    /*-----���仯������������-----*/
    if(fec <= ecRule[0])
    {
      ecFuzzy[0] =1.0;
      pec = 0;
    }
    else if(fec < ecRule[1])
    {
      ecFuzzy[0] = (ecRule[1] - fec)/(ecRule[1]-ecRule[0]);
      pec = 0 ;
    }
    else if(fec < ecRule[2])
    {
      ecFuzzy[0] = (ecRule[2] - fec)/(ecRule[2]-ecRule[1]);
      pec = 1;
    }
    else if(fec < ecRule[3])
    {
      ecFuzzy[0] = (ecRule[3] - fec)/(ecRule[3]-ecRule[2]);
      pec = 2 ;
    }
    else if(fec < ecRule[4])
    { 
      ecFuzzy[0] = (ecRule[4] - fec)/(ecRule[4]-ecRule[3]);
      pec=3;
    }	
    else										
    {
      ecFuzzy[0] =1.0;
      pec=4;
    }
    ecFuzzy[1] = 1.0 - ecFuzzy[0];
}

/*******************************************************************************
 *  @brief      fuzzy_query����
 *  @note       ��ѯģ��������������ĸ�ռ��
                
 *  @warning    18/3/10 ����ο�binary-star��  v3.0
 ******************************************************************************/
void fuzzy_query(void)//��ѯģ�������
{
    for(l = 0;l <= rank; l++) //��������Ա��ۼ�
    {
        Fuzzy_kp[l] = 0;
        Fuzzy_kd[l] = 0;
    }
      /*��ѯkpģ�������*/  
    num = rule_kp[pe][pec];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[0];

    num = rule_kp[pe][pec+1];
    Fuzzy_kp[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = rule_kp[pe+1][pec];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = rule_kp[pe+1][pec+1];
    Fuzzy_kp[num] += eFuzzy[1] * ecFuzzy[1];
    
      /*��ѯkdģ�������*/  
    num = rule_kd[pe][pec];
    Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[0];

    num = rule_kd[pe][pec+1];
    Fuzzy_kd[num] += eFuzzy[0] * ecFuzzy[1];	
    
    num = rule_kd[pe+1][pec];
    Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[0];
    
    num = rule_kd[pe+1][pec+1];
    Fuzzy_kd[num] += eFuzzy[1] * ecFuzzy[1];
}

/*******************************************************************************
 *  @brief      fuzzy_solve����
 *  @note       ���ķ���ģ��
                
 *  @warning    18/3/10 ����ο�binary-star�� v3.0
 ******************************************************************************/
void fuzzy_solve(void)//��ģ���õ�pdֵ
{
    steer_P = 0; //���P��Dֵ�Ա��ۼ�
    steer_D = 0;
    /*������ķ���ģ��*/
    for(l = 0;l < rank; l++)
    {
      steer_P += Fuzzy_kp[l] * Rule_kp[l];
      steer_D += Fuzzy_kd[l] * Rule_kd[l];   
    }
}

/*******************************************************************************
 *  @brief      steercontrol����
 *  @note       ���ת�Ǻ���
                
 *  @warning    18/3/10 v3.0
 ******************************************************************************/
void steercontrol(void) 
{
    if(steer_P < 0)  steer_P = -steer_P; //������� P ֵ��Ϊ������������������ǡ��Ϊ���ת�ǵ�����
    if(steer_D < 0)  steer_D = -steer_D; //������� D ֵ��Ϊ����
    steerctrl_error = (int)( P_power * steer_P * fe + D_power * steer_D * (fe - fe_last) );//���ת������
    last_steerctrl = steerctrl;
    steerctrl = Midsteering + steerctrl_error; //���ת��PWM
}
    


    