#include<reg52.h>
#define uchar unsigned char
#define uint unsigned int

void timer_int(void); 	//定时器初始化函数
char read(void);				//读红外对管函数
int  pid(char);         //PID程序

/*-------------------------	位定义-------------------------*/
/*H桥开关位定义*/
sbit LeftUp2 = P1^0;
sbit LeftUp1 = P1^1;
sbit LeftDown2 = P1^2;
sbit LeftDown1 = P1^3;
sbit RightDown2 = P1^4;
sbit RightDown1 = P1^5;
sbit RightUp2 = P1^6;
sbit RightUp1 = P1^7;
sbit top = P3^6;

uchar Kp=7;	        //比例系数 7
float Ki=5;         //积分系数(100倍值) 10表示0.1
uchar Kd=5;	        //微分系数5
uchar PWM_MAX=100;  //PWM控制级数
uchar SPEED;        //速度因子 0-10取值
uchar SPD_SET=10;   //速度设定值
uchar DIF_TIME=2;   //微分项作用时长2

/*------------------------全局变量定义---------------------*/
uchar Reversion_L=0;  //左轮反转时长
uchar Reversion_R=0;  //右轮反转时长
uchar PWM_L=0 ;    		//左电机的占空比
uchar PWM_R=0 ;   	 	//右电机的占空比
uchar timer_count=0; 	//定时器中断执行次数
uchar pid_time=0;     //pid调整间隔时长
char  position=0x10;  //车体偏移状态，初始化为停车状态
char  error=0;
char  last_error=0;   //前一误差值
uchar dif_act_time;   //微分项作用时长
int   I_sum=0;        //积分项累计数（100倍）
int   I_term=0;       //积分项
int   P_term, D_term; //比例项和微分项

/*定时器初始化程序*/
void timer_int(void)
{
	IE = 0x82;		//1XX0 0010 允许中断 无定时器1中断 定时器0中断
	TMOD = 0x02; 	//使用定时器0，工作方式2 自装入八位计时
	TL0 = 6;			//计时0.25ms
  TH0 = 6;
	TR0 = 1;			//开计时器0
	IT0 = 0; 			//低电平触发方式
	EX0 = 1; 
}

/*定时器0中断服务程序*/
void inter_0(void)	interrupt 1	//定时器0的中断服务程序
{
/*-------------------------PWM调制-------------------------*/		
    if(timer_count<PWM_L)
    {		
      if(Reversion_L)
      {	//左向反转	
        LeftUp1=0;LeftUp2=1;
        LeftDown1=0;LeftDown2=1;
      }
      else
      { //左向正转
        LeftUp1=1;LeftUp2=0;
        LeftDown1=1;LeftDown2=0;
      }
    }
    else
    { //左向停止
      LeftUp1=0;LeftUp2=0;
      LeftDown1=0;LeftDown2=0;	
    }
    if(timer_count<PWM_R)
    {		
      if(Reversion_R)
      {	//右向反转	
        RightUp1=0;RightUp2=1;
        RightDown1=0;RightDown2=1;
      }
      else
      { //右向正转
        RightUp1=1;RightUp2=0;
        RightDown1=1;RightDown2=0;
      }
    }
    else
    { //右向停止
      RightUp1=0;RightUp2=0;
      RightDown1=0;RightDown2=0;	
    }
    
/*-------------------------PWM调制计数-------------------------*/	
	timer_count++;
	if(timer_count>=PWM_MAX)
  { timer_count=0;
    pid_time++;
  }     
}

/*PID控制参数计算程序*/
int pid(char error)
{
	int pid_out;
  
  I_sum+=Ki*error;	          //积分量计算
  I_term=I_sum/100;
	
  if(I_term>PWM_MAX/2) 
  { 
		I_term=PWM_MAX/2;	          //限定积分量上限
    I_sum=100*PWM_MAX/2;
  }
  else if(I_term<-(PWM_MAX/2)) 
  { 
		I_term=-(PWM_MAX/2);	      //限定积分量下限
    I_sum=-100*PWM_MAX/2;
  }
  
	if(error!=last_error)
  {
    D_term =Kd*(error - last_error); 	//微分量计算
    dif_act_time=DIF_TIME;            //设定微分项作用时长
    
    //if(error==0) I_term+=P_term;
    P_term =Kp*error;	//比例量计算
    
    last_error=error;	//缓存当前误差量
  }
  if(dif_act_time > 0) dif_act_time--;
  else D_term=0;      //微分项作用到时
  
	pid_out= (int)(P_term+I_term+D_term);	      //PID控制量计算
	if(pid_out>PWM_MAX) pid_out=PWM_MAX;		    //控制量上限=PWM_MAX
	else if(pid_out<-PWM_MAX) pid_out=-PWM_MAX;	//控制量下限=-PWM_MAX
  
  return(pid_out);
}

/*主程序*/
void main()
{
  int i;
  int pid_control=0;
  uchar state;
  
	P1 = 0x00;					//小车静止
	timer_int(); 				//定时器初始化
	while(1)
	{    
    
		while(top==0)
		{
			P1=0x00;
		}
		//读红外寻迹对管,返回车体位置误差值：右+ 左- 范围(-10~10,-12,12,0x10) 
    state=P0;
    switch(state)
    {
      case 0x00: if((position>=-1 && position<=1) || (position==0x10))
                 { 
                    position=0;   //车体回居中位置
                 }
                 if(position<=-11) //左转不足出赛道
                 {  
                    position=-13;                   
                 }  
                 if(position>=11)  //右转不足出赛道
                 {  
                    position=13;
                 }   
                 break;	//全白 前进
      
      case 0x18: position=0;break;  //车体居中
      
      case 0x01: position=11;break;	//右一
      case 0x03: position=9;break;	//右一、二
      case 0x02: position=7;break;	//右二
      case 0x06: position=5;break;	//右二、三
      case 0x04: position=3;break;	//右三
      case 0x0C: position=2;break;	//右三、四
      case 0x08: position=1;break;	//右四
      
      case 0x80: position=-11;break;//左一
      case 0xC0: position=-9;break;	//左一、二
      case 0x40: position=-7;break;	//左二
      case 0x60: position=-5;break;	//左二、三
      case 0x20: position=-3;break;	//左三
      case 0x30: position=-2;break;	//左三、四
      case 0x10: position=-1;break;	//左四
      
      default:  if(position!=0x10) 
                 { //非法状态
                   position=0x10;
                   PWM_L =  0;
                   PWM_R =  0; //停止
                 }
                 break;	
      }
      error=position;
      if(error!=0x10)//合法状态
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