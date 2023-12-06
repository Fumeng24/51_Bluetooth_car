#include <REGX52.H>

sfr  P4=0xe8;//头文件未定义P4口，由于IO口损坏过多被迫启用P4口
sbit INT11=P1^1;
sbit INT2=P1^2;
sbit INT3=P1^3;
sbit INT4=P1^4;
sbit EN1=P1^0;
sbit EN2=P1^5;
sbit left_led1=P1^7;
sbit left_led2=P1^6;
sbit right_led1=P3^2;
sbit right_led2=P3^3;
sbit Echo=P3^7;
sbit Trig=P3^6;
bit  flag =0;
bit isBTControl=1;
#define left_motor_stops  EN1=0
#define right_motor_stops  EN2=0
#define left_motor_go  INT11=0,INT2=1
#define left_motor_back  INT11=1,INT2=0
#define right_motor_go  INT3=0,INT4=1
#define right_motor_back  INT3=1,INT4=0

unsigned int  time=0;
unsigned char PWMR=0,PWML=0,t=0;
unsigned char SR04_Time1=0;
unsigned char SR04_Time2=0;
unsigned char receive_data;//接收到的数据
unsigned int EchoCnt;//超声波返回计时
int trackingMode = 0;//标志变量

//延迟函数
void DelayUs(unsigned int i)
{
	while(--i);
}

//前进
void forward()
{
	PWML=50;
	PWMR=50;
	left_motor_go; 
	right_motor_go; 
}

//后退
void backward()
{
	PWML=50;
	PWMR=50;
	left_motor_back;
	right_motor_back;
}

//左前方
void slow_forward_left()
{
	PWMR=35;
	left_motor_stops;
	right_motor_go; 
}

//右前方
void slow_forward_right()
{
	PWML=35;
	right_motor_stops;
	left_motor_go;  
}

//左后方
void slow_back_left()
{
	PWMR=35;
	right_motor_back;
	left_motor_stops;  
}

//右后方
void slow_back_right()
{
	PWML=35;
	left_motor_back;
	right_motor_stops; 
}

//原地左转
void fast_left()
{
	PWMR=35;
	left_motor_back;
	PWML=35;
	right_motor_go; 
}

//原地右转
void fast_right()
{
	PWMR=35;
	left_motor_go;
	PWML=35;
	right_motor_back; 
}

//停止
void stop()
{
	PWML=0;
	PWMR=0;
	left_motor_stops;
	right_motor_stops;
}

//定时器0-PWM
void Timer0_Init()
{
	TMOD|=0X01;
	TH0=(65536-100)/256;
	TL0=(65536-100)%256;
	TR0=1;
	ET0=1;
	EA=1;
}

//定时器1-蓝牙
void Timer1_Init()
{
	TMOD &= 0x0F; // 清除定时器1模式位
	TMOD |= 0x20; // 设置定时器1为模式2（8位自动重载定时/计数器）
	TH1 = 0xFD;   // 定时器1初值，用于9600bps
	TL1 = 0xFD;   // 定时器1初值，用于9600bps
	TR1 = 1;      // 启动定时器1
	SCON = 0x50;  // 设置串行口为模式1（8位数据, 可变波特率）
}

//定时器2-超声波
void HC04_Init(void)
{
	T2MOD=0;				//初始化模式寄存器
	T2CON=0;				//初始化控制寄存器
	TH2=0;
	TL2=0;          
	ET2=0;           	 	//允许T2中断
	EA=1;			 		//开启总中断
}

//中断设置
void Interrupt_Init()
{
	EA = 1;       // 开启全局中断
	ES = 1;       // 开启串行中断
	PT1 = 1;      // 定时器1（串行中断）优先级高
	PT0 = 0;      // 定时器0优先级低
}

//蓝牙串口处理数据控制小车
void Bluetooth_Control()
{
	switch (receive_data)
	{
		case '1':
			forward(); break;
		case '2':
			backward(); break;
		case '3':
			fast_right(); break;
		case '4':
			fast_left(); break;
		case '5':
			slow_forward_left(); break;
		case '6':
			slow_forward_right(); break;
		case '7':
			slow_back_left(); break;
		case '8':
			slow_back_right(); break;
		case '0':
			stop(); break;
	}
}

//超声波算法获取距离
void HC04_Loop(void)
{  
	Trig = 1;
	DelayUs(20);
	Trig = 0;
	while(!Echo);		//当RX（ECHO信号回响）为零时等待
	TR2=1;	    		//开启计数
	while(Echo);		//当RX为1计数并等待
	TR2=0;				//关闭计数
	time=TH2*256+TL2;
	TH2=0;
	TL2=0;
	EchoCnt=(float)(time*1.085)*0.17;     //单位是毫米
}

//循迹算法
void xunji()
{
	unsigned char jiaodu;
	if(left_led2==0&&left_led1==0&&right_led1==0&&right_led2==0)//0000
	{
		switch(jiaodu)
		{
			case 0:forward();break;
			case 1:fast_left();break;
			case 2:fast_right();break;
			default:forward();break;
		}
	}
	if(left_led2==0&&left_led1==1&&right_led1==0&&right_led2==0)//0100
	{
		slow_forward_left();
		jiaodu=0;
	}
	if(left_led2==0&&left_led1==0&&right_led1==1&&right_led2==0)//0010
	{ 
		slow_forward_right();
		jiaodu=0;
	}
	if(left_led2==1&&left_led1==0&&right_led1==0&&right_led2==0)//1000
	{
		fast_left();
		jiaodu=1;
	}
	if(left_led2==0&&left_led1==0&&right_led1==0&&right_led2==1)//0001
	{
		fast_right();
		jiaodu=2;
	}	
	if(left_led2==1&&left_led1==1&&right_led1==0&&right_led2==0)//1100
	{
		fast_left();
		jiaodu=1;
	}
	if(left_led2==0&&left_led1==0&&right_led1==1&&right_led2==1)//0011
	{
		fast_right();
		jiaodu=2;
	}
	if(left_led2==1&&left_led1==1&&right_led1==1&&right_led2==1)//1111
	{
		stop();
		jiaodu=3;
	}
}

//超声波距离处理数据控制小车
void HC04_Control()
{
		// 当物体距离大于等于上限时，物体前进
		if (EchoCnt >= 200)
		{
			xunji();
		}
		// 当物体距离小于下限时，物体后退
		else if (EchoCnt < 150)
		{
			backward();
		}
		// 当物体距离在范围内时，不进行任何操作
		else
		{
			stop();
		}
}
//主函数
void main()
{
	receive_data = 0;
	Timer0_Init();
	Timer1_Init();
	HC04_Init();
	Interrupt_Init();
	while(1)
	{	
		if (trackingMode)
		{
//			xunji();
			HC04_Control();
			isBTControl=0;
		}
		if (!trackingMode)
		{
			//如果不是循迹模式，则正常处理其他命令
			Bluetooth_Control();
			isBTControl=1;
		}
	}
}











//PWM信号产生服务程序
void Timer0_interrupt()  interrupt 1
{
	TH0=(65536-100)/256;
	TL0=(65536-100)%256;
	t++;
	if(t<PWML)
	EN1=1;
	else
		EN1=0;
	if(t<PWMR)
		EN2=1;
	else
		EN2=0;
	if(t>=100)
	{
		t=0;
	}
	if(isBTControl==0)
	{
		SR04_Time1++;
		if(SR04_Time1==255)
		{
			SR04_Time2++;
			SR04_Time1=0;
			if(SR04_Time2==10)
			{
				HC04_Loop();
				SR04_Time2=0;				
			}

		}
	}
}

//蓝牙串行中断服务程序
void Serial_ISR(void) interrupt 4
{
	if (RI)
	{
		RI = 0;//清除接收中断标志
		receive_data = SBUF;
		if (receive_data == '9')
		{
			trackingMode = !trackingMode;//切换模式
		}
	}
	if (TI)
	{
		TI = 0;//清除发送中断标志
	}
}



