#include <REGX52.H>

sfr  P4=0xe8;//ͷ�ļ�δ����P4�ڣ�����IO���𻵹��౻������P4��
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
unsigned char receive_data;//���յ�������
unsigned int EchoCnt;//���������ؼ�ʱ
int trackingMode = 0;//��־����

//�ӳٺ���
void DelayUs(unsigned int i)
{
	while(--i);
}

//ǰ��
void forward()
{
	PWML=50;
	PWMR=50;
	left_motor_go; 
	right_motor_go; 
}

//����
void backward()
{
	PWML=50;
	PWMR=50;
	left_motor_back;
	right_motor_back;
}

//��ǰ��
void slow_forward_left()
{
	PWMR=35;
	left_motor_stops;
	right_motor_go; 
}

//��ǰ��
void slow_forward_right()
{
	PWML=35;
	right_motor_stops;
	left_motor_go;  
}

//���
void slow_back_left()
{
	PWMR=35;
	right_motor_back;
	left_motor_stops;  
}

//�Һ�
void slow_back_right()
{
	PWML=35;
	left_motor_back;
	right_motor_stops; 
}

//ԭ����ת
void fast_left()
{
	PWMR=35;
	left_motor_back;
	PWML=35;
	right_motor_go; 
}

//ԭ����ת
void fast_right()
{
	PWMR=35;
	left_motor_go;
	PWML=35;
	right_motor_back; 
}

//ֹͣ
void stop()
{
	PWML=0;
	PWMR=0;
	left_motor_stops;
	right_motor_stops;
}

//��ʱ��0-PWM
void Timer0_Init()
{
	TMOD|=0X01;
	TH0=(65536-100)/256;
	TL0=(65536-100)%256;
	TR0=1;
	ET0=1;
	EA=1;
}

//��ʱ��1-����
void Timer1_Init()
{
	TMOD &= 0x0F; // �����ʱ��1ģʽλ
	TMOD |= 0x20; // ���ö�ʱ��1Ϊģʽ2��8λ�Զ����ض�ʱ/��������
	TH1 = 0xFD;   // ��ʱ��1��ֵ������9600bps
	TL1 = 0xFD;   // ��ʱ��1��ֵ������9600bps
	TR1 = 1;      // ������ʱ��1
	SCON = 0x50;  // ���ô��п�Ϊģʽ1��8λ����, �ɱ䲨���ʣ�
}

//��ʱ��2-������
void HC04_Init(void)
{
	T2MOD=0;				//��ʼ��ģʽ�Ĵ���
	T2CON=0;				//��ʼ�����ƼĴ���
	TH2=0;
	TL2=0;          
	ET2=0;           	 	//����T2�ж�
	EA=1;			 		//�������ж�
}

//�ж�����
void Interrupt_Init()
{
	EA = 1;       // ����ȫ���ж�
	ES = 1;       // ���������ж�
	PT1 = 1;      // ��ʱ��1�������жϣ����ȼ���
	PT0 = 0;      // ��ʱ��0���ȼ���
}

//�������ڴ������ݿ���С��
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

//�������㷨��ȡ����
void HC04_Loop(void)
{  
	Trig = 1;
	DelayUs(20);
	Trig = 0;
	while(!Echo);		//��RX��ECHO�źŻ��죩Ϊ��ʱ�ȴ�
	TR2=1;	    		//��������
	while(Echo);		//��RXΪ1�������ȴ�
	TR2=0;				//�رռ���
	time=TH2*256+TL2;
	TH2=0;
	TL2=0;
	EchoCnt=(float)(time*1.085)*0.17;     //��λ�Ǻ���
}

//ѭ���㷨
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

//���������봦�����ݿ���С��
void HC04_Control()
{
		// �����������ڵ�������ʱ������ǰ��
		if (EchoCnt >= 200)
		{
			xunji();
		}
		// ���������С������ʱ���������
		else if (EchoCnt < 150)
		{
			backward();
		}
		// ����������ڷ�Χ��ʱ���������κβ���
		else
		{
			stop();
		}
}
//������
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
			//�������ѭ��ģʽ��������������������
			Bluetooth_Control();
			isBTControl=1;
		}
	}
}











//PWM�źŲ����������
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

//���������жϷ������
void Serial_ISR(void) interrupt 4
{
	if (RI)
	{
		RI = 0;//��������жϱ�־
		receive_data = SBUF;
		if (receive_data == '9')
		{
			trackingMode = !trackingMode;//�л�ģʽ
		}
	}
	if (TI)
	{
		TI = 0;//��������жϱ�־
	}
}



