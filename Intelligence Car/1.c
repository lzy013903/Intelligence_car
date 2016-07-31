#include<reg52.h>
#define uchar unsigned char
#define uint unsigned int

void timer_int(void); 	//��ʱ����ʼ������
char read(void);				//������Թܺ���
int  pid(char);         //PID����

/*-------------------------	λ����-------------------------*/
/*H�ſ���λ����*/
sbit LeftUp2 = P1^0;
sbit LeftUp1 = P1^1;
sbit LeftDown2 = P1^2;
sbit LeftDown1 = P1^3;
sbit RightDown2 = P1^4;
sbit RightDown1 = P1^5;
sbit RightUp2 = P1^6;
sbit RightUp1 = P1^7;
sbit top = P3^6;

uchar Kp=7;	        //����ϵ�� 7
float Ki=5;         //����ϵ��(100��ֵ) 10��ʾ0.1
uchar Kd=5;	        //΢��ϵ��5
uchar PWM_MAX=100;  //PWM���Ƽ���
uchar SPEED;        //�ٶ����� 0-10ȡֵ
uchar SPD_SET=10;   //�ٶ��趨ֵ
uchar DIF_TIME=2;   //΢��������ʱ��2

/*------------------------ȫ�ֱ�������---------------------*/
uchar Reversion_L=0;  //���ַ�תʱ��
uchar Reversion_R=0;  //���ַ�תʱ��
uchar PWM_L=0 ;    		//������ռ�ձ�
uchar PWM_R=0 ;   	 	//�ҵ����ռ�ձ�
uchar timer_count=0; 	//��ʱ���ж�ִ�д���
uchar pid_time=0;     //pid�������ʱ��
char  position=0x10;  //����ƫ��״̬����ʼ��Ϊͣ��״̬
char  error=0;
char  last_error=0;   //ǰһ���ֵ
uchar dif_act_time;   //΢��������ʱ��
int   I_sum=0;        //�������ۼ�����100����
int   I_term=0;       //������
int   P_term, D_term; //�������΢����

/*��ʱ����ʼ������*/
void timer_int(void)
{
	IE = 0x82;		//1XX0 0010 �����ж� �޶�ʱ��1�ж� ��ʱ��0�ж�
	TMOD = 0x02; 	//ʹ�ö�ʱ��0��������ʽ2 ��װ���λ��ʱ
	TL0 = 6;			//��ʱ0.25ms
  TH0 = 6;
	TR0 = 1;			//����ʱ��0
	IT0 = 0; 			//�͵�ƽ������ʽ
	EX0 = 1; 
}

/*��ʱ��0�жϷ������*/
void inter_0(void)	interrupt 1	//��ʱ��0���жϷ������
{
/*-------------------------PWM����-------------------------*/		
    if(timer_count<PWM_L)
    {		
      if(Reversion_L)
      {	//����ת	
        LeftUp1=0;LeftUp2=1;
        LeftDown1=0;LeftDown2=1;
      }
      else
      { //������ת
        LeftUp1=1;LeftUp2=0;
        LeftDown1=1;LeftDown2=0;
      }
    }
    else
    { //����ֹͣ
      LeftUp1=0;LeftUp2=0;
      LeftDown1=0;LeftDown2=0;	
    }
    if(timer_count<PWM_R)
    {		
      if(Reversion_R)
      {	//����ת	
        RightUp1=0;RightUp2=1;
        RightDown1=0;RightDown2=1;
      }
      else
      { //������ת
        RightUp1=1;RightUp2=0;
        RightDown1=1;RightDown2=0;
      }
    }
    else
    { //����ֹͣ
      RightUp1=0;RightUp2=0;
      RightDown1=0;RightDown2=0;	
    }
    
/*-------------------------PWM���Ƽ���-------------------------*/	
	timer_count++;
	if(timer_count>=PWM_MAX)
  { timer_count=0;
    pid_time++;
  }     
}

/*PID���Ʋ����������*/
int pid(char error)
{
	int pid_out;
  
  I_sum+=Ki*error;	          //����������
  I_term=I_sum/100;
	
  if(I_term>PWM_MAX/2) 
  { 
		I_term=PWM_MAX/2;	          //�޶�����������
    I_sum=100*PWM_MAX/2;
  }
  else if(I_term<-(PWM_MAX/2)) 
  { 
		I_term=-(PWM_MAX/2);	      //�޶�����������
    I_sum=-100*PWM_MAX/2;
  }
  
	if(error!=last_error)
  {
    D_term =Kd*(error - last_error); 	//΢��������
    dif_act_time=DIF_TIME;            //�趨΢��������ʱ��
    
    //if(error==0) I_term+=P_term;
    P_term =Kp*error;	//����������
    
    last_error=error;	//���浱ǰ�����
  }
  if(dif_act_time > 0) dif_act_time--;
  else D_term=0;      //΢�������õ�ʱ
  
	pid_out= (int)(P_term+I_term+D_term);	      //PID����������
	if(pid_out>PWM_MAX) pid_out=PWM_MAX;		    //����������=PWM_MAX
	else if(pid_out<-PWM_MAX) pid_out=-PWM_MAX;	//����������=-PWM_MAX
  
  return(pid_out);
}

/*������*/
void main()
{
  int i;
  int pid_control=0;
  uchar state;
  
	P1 = 0x00;					//С����ֹ
	timer_int(); 				//��ʱ����ʼ��
	while(1)
	{    
    
		while(top==0)
		{
			P1=0x00;
		}
		//������Ѱ���Թ�,���س���λ�����ֵ����+ ��- ��Χ(-10~10,-12,12,0x10) 
    state=P0;
    switch(state)
    {
      case 0x00: if((position>=-1 && position<=1) || (position==0x10))
                 { 
                    position=0;   //����ؾ���λ��
                 }
                 if(position<=-11) //��ת���������
                 {  
                    position=-13;                   
                 }  
                 if(position>=11)  //��ת���������
                 {  
                    position=13;
                 }   
                 break;	//ȫ�� ǰ��
      
      case 0x18: position=0;break;  //�������
      
      case 0x01: position=11;break;	//��һ
      case 0x03: position=9;break;	//��һ����
      case 0x02: position=7;break;	//�Ҷ�
      case 0x06: position=5;break;	//�Ҷ�����
      case 0x04: position=3;break;	//����
      case 0x0C: position=2;break;	//��������
      case 0x08: position=1;break;	//����
      
      case 0x80: position=-11;break;//��һ
      case 0xC0: position=-9;break;	//��һ����
      case 0x40: position=-7;break;	//���
      case 0x60: position=-5;break;	//�������
      case 0x20: position=-3;break;	//����
      case 0x30: position=-2;break;	//��������
      case 0x10: position=-1;break;	//����
      
      default:  if(position!=0x10) 
                 { //�Ƿ�״̬
                   position=0x10;
                   PWM_L =  0;
                   PWM_R =  0; //ֹͣ
                 }
                 break;	
      }
      error=position;
      if(error!=0x10)//�Ϸ�״̬
      {
        if(pid_time>=1 || error!=last_error)    //timer_count%25==1
        {
          pid_time=0;
        
          if(error>=0) SPEED=SPD_SET;//-error*0.2;
          else SPEED=SPD_SET;//+error*0.2;
          
          pid_control=pid(error);
          if(pid_control>=0)
          {
            i=  SPEED*(PWM_MAX + pid_control)/10;
            PWM_L =  i>PWM_MAX ? PWM_MAX: i;
            Reversion_L=0;
            
            i=  SPEED*(PWM_MAX - 2*pid_control)/10;
            if(i<0)  
            { Reversion_R=1; i=-i;}
            else Reversion_R=0;
            PWM_R =  i>PWM_MAX ? PWM_MAX: i;            
          }
          else
          {
            i=  SPEED*(PWM_MAX + 2*pid_control)/10;
            if(i<0)  
            { Reversion_L=1; i=-i;}
            else Reversion_L=0;
            PWM_L =  i>PWM_MAX ? PWM_MAX: i;
            
            i=  SPEED*(PWM_MAX - pid_control)/10;
            PWM_R =  i>PWM_MAX ? PWM_MAX: i;
            Reversion_R=0;
          }
        }
      }
  }		
}