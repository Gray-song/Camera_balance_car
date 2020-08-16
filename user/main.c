
#include "flash.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
////#include "dma.h"
#include "ov7725.h"
#include "i2c.h"
#include  <OLED.h>
#include <string.h>
#include "gpio.h"
#include "common.h"
#include "i2c.h"
#include "uart.h"
#include "adc.h"
#include "pit.h"
#include "outputdata.h"
#include "math.h"
#include "ftm.h"
#include "OLED.h"
#include "systick.h"
#include "stdlib.h"
#include <stdlib.h>

#define White_Line_Min 20
#define White_Line_Max 80 
#define MOTOR_DEAD_VAL_L_ 550 // 940//����������ѹ
#define MOTOR_DEAD_VAL_R_ 600// 940//����������ѹ
#define	Speed_JiYian 5000    //��������ٶ�

//#define	MMA7361_z_states 1860	//1858//1840//1860//1989//1994    //���ٶ�z����ƫֵ  1985 p=1 d=78;1985
//#define	MMA7361_x_states	1868//1955//2018//1868//662087//2024    //���ٶ�x����ƫֵ1751 1855
#define angle_rate 228.59//280//228.59//282.79        //������ϵ��

//���ȶ���λ�����ĺ궨��
///////////////����//////////////////////////
#define KEY1  PBin(20)  //����PTB�˿ڵ�20����Ϊ����
#define KEY2  PBin(21)
#define KEY3  PBin(22)
#define KEY4  PBin(23)
#define KEY5  PCin(5)

#define	KEY_SINGLE	(0x00)    //����״̬����ʾ��������
u8 key;                       //ȡֵΪ1 2 3 4 5�ֱ������·��������
///////////////////////////////////////////////
#define DIP1  PAin( 4)
#define DIP2  PAin( 5)
#define DIP3  PAin(12)
#define DIP4  PAin(13)

#define ON	(0X01)
#define OFF	(0X00)

#define	DIP_SINGLE (0X00)     //���뿪��״̬����ʾ���رպ�
u8 dip1,dip2,dip3,dip4;       //�ֱ������·���뿪��
/////////////////led��/////////////////////////
#define LED1  PAout(14)  	//D1��
#define LED2  PAout(15)		//D2��
#define LED3  PAout(16)		//D3��
#define LED4  PAout(17)		//D4��
#define BEEP  PCout(6)		//BEEP
#define LED_ON (0X00)  	//LED�ƿ�
#define LED_OFF	(0X01)  	//LED�ƹ�
#define BEEP_ON	(0X01)  	//BEEP��
#define BEEP_OFF (0X00)  //BEEP��
/////////////////////////////////////////////////
#define SQM_R  PBin(16)//�����������������
#define SQM_L  PBin(19)

/////////////////////��������////////////////////////////////////

void Rd_Ad_Value(void);
void AD_Calculate(void);//��һ������
void KEY_Init(void);   //������ʼ������
void Key_Scan(void);   //����ɨ�躯��

void DIP_Init(void);		//���뿪�س�ʼ������
void Dip_Scan(void);		//���뿪��ɨ�躯��
void Led_Init(void);		//LED�Ƴ�ʼ������
void Beep_Init(void);	//��������ʼ��
void Uart_Init(void);	//���ڳ�ʼ��
void Tuo_Luo_Yi_Init(void);		//�����ǳ�ʼ������
void Motor_Init(void);					//�����ʼ������
void Key(void); //��������
void Oled_show(void);//oled��ʾ

void Go_Speed(int Speed_L,int	Speed_R);//����ٶȺ���
void Speed_Calculate(float angle,float angle_dot);//ֱ���ٶȿ��ƺ���
void SMQ_Init(void);//��������ʼ��
static void UART_RX_ISR(uint16_t byteReceived);//�����жϻص���������
void UpdateSpeed(void);//�ٶȻ����ƺ���
void OV_ISR(uint32_t index);
void Tu_Xiang(void);
void Find_Mid_Line(u8  image_dec[OV7725_H][OV7725_W]);//����ͷѰ������
void pit_Init(void);
void SXT(void);//����
void DirControlOutput(void);//����ƽ�����
void SPControlOutput(void);//�ٶ�ƽ�����
void Stopcar(void);//ͣ������
void QingHua_AngleCalaulate(float G_angle,float Gyro);//�廪����
void Derror(void);
///////////////////////////�������////////////////////////////////

uint8_t gImageBuffer[OV7725_SIZE];   //����ͷԭʼ����
uint8_t image_dec[OV7725_H][OV7725_W];  //ͼ���ѹ�������

u16 MMA7361_z,MMA7361_x,ENC03_y,ENC03_x;//���ٶȼ�z��ADֵ�����ٶȼ�x��ADֵ��������y��ADֵ

static float g_fCarAngle,g_fGyroscopeAngleIntegral;

static float ACC_z,ACC_x,GYRO_y,angle_ACC,angle_GYRO_y;//���ٶȼ�z���ȥ��ֵ��ֵ�����ٶȼ�x���ȥ��ֵ��ֵ��������y���ȥ��ֵ��ֵ
//static float Ang_Deviation;//ƽ��λ�ý�
//float Ang_Zero=0;//�Ƕ����ƫ��
//volatile float  Speed_L,Speed_R,speed_Start,Speed_L_Last,Speed_R_Last;  //ֱ���ٶȲ���

static int GYRO_VAL = 0;//�����ǳ�ֵ
static int GYRO_VAL_x = 2036;//������x���ֵ

static int Speed_L,Speed_R,speed_Start;//������ֱ���ٶ�
unsigned int speed_num = 0;
unsigned int num = 0;//���ƺ�������ʱ����Լӱ���
unsigned int PIT_Flog = 0;
unsigned int SXT_Timer = 0;
unsigned int tuxiang_Timer = 0;

uint8_t Stop_Car_Time = 0;
u8 key_flag; //�����Լӱ���
float k;//�������ȱ���
u8 k5_flag = 0;
u8 k5_flag_ = 0;
bool flag;
static double P_ANGLE = 0.232;//0.1;//0.034;//0.032;//0.19798;//0.1986195;//0.1979895;//0.5979895;//0.1979895;
static double D_ANGLE = 0.285;//0.467;//0.30199;//0.185;//2.04601;//4.0473972;//2.0460172;//5.7460172;//4.9460172;
static float Sudu_P = 0, sudu_P = 0.102 ;//�ٶȱ�������   0
static float Sudu_I = 0, sudu_I = 0.029;//�ٶȻ��ֲ���
static float Sudu_D = 0, sudu_D = 0.01 ;//�ٶ�΢�ֲ���  

static int suduL, suduR, CarSpeed, setspeed = 1510, g_HighestSpeed = 1000 , g_LowestSpeed = 1010, Setspeed = 0, G_HighestSpeed = 0;
static float SpeedError, LastError, offset2, dError,Speed_out, LastdError;
static int WheelError;
float SP_out_old = 0,SP_out_new = 0,SP_out = 0,SP_ControlPeriod = 0;

int MMA7361_z_states = 1525;//1880;//1858;
int MMA7361_x_states = 1965;//1868;//1955;
/////������ر���
int middle = 40;
float fx_sxt = 0,fx_gy = 0, fx_out = 0;
static double FX_P = 0, fx_p = 165.0957;
static double FX_D = 0, fx_d = 1.6008;
float Dir_new = 0, Dir_old = 0, Dir_OUT = 0, Dir_ControlPeriod = 0;
int derror;//����ƫ��
float fP,fI,fSpeedControlIntegral ;
///////////////////////////////���մ���Flash�е����ݱ���/////////////////
uint32_t P_ANGLE_Flash = 0, D_ANGLE_Flash = 0;
uint32_t sudu_P_Flash = 0, sudu_I_Flash = 0, sudu_D_Flash = 0;
uint32_t MMA7361_z_states_Flash = 0;
uint32_t setspeed_Flash = 0, g_HighestSpeed_Flash = 0;
uint32_t fx_p_Flash = 0, fx_d_Flash = 0;
unsigned int StartCarTimer = 0;//�ȴ�����ʱ��
u8 stopcar_flag = 0;
float a;
uint8_t H,W,Left = 0, Right = 0, Mid = 0;
uint8_t Left_Old = 0, Right_Old = 80;
uint8_t Left_flag = 0, Right_flag = 0;
uint8_t Mid_line[30];
uint8_t yuan_jishu = 0;
uint8_t yuan_flag = 0;
uint8_t zijia = 10;

uint8_t zijia_l = 10;
uint8_t zijia_r = 10;
uint8_t L_barriar = 0, R_barriar = 0;
uint8_t qipaoxian = 0;
uint32_t StopCarJishi = 0;
void Qipaoxian(void);
///////////////////////////////////////

float quanzhi_sum = 0;
float mid_line_sum = 0;
float mid_line_mean = 0;
int row = 0, line = 0;			
int suduL_temp[16], suduR_temp[16], WheelError_temp[3];
int Carspeed_temp[3],ActualInstantaneous[3];							
//	///////////////////////////////////////
	///////////������////////////////////
int anglez_temp[3] = {0}, gyroy_temp[3] = {0};
int anglez,gyroy;
float Q_angle = 0.01; //0.001   //????????
float Q_gyro = 0.0003;  //0.0003   //0.003???????  0.00003???
float R_angle = 0.0001;  //0.0001  //0.01???   0.001???

float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float y, S; float K_0, K_1;
double KalmanFilter(double angleout , double newAngle, double newRate,double dt);									
///////////////////////////////////////////////////
uint8_t EdgeLeft[60] = {0},EdgeRight[60] = {0}, centerline[60] = {0}, EdgeLeftceshi[60] = {0}, EdgeRightceshi[60] = {0}, centerlineceshi[60] = {0};
#define LL 0
#define RL 80
#define LINE_MAX 80      //???????,??   ???? .h ?????????
#define ROW_MAX 60   
int quanzhi[60]=
{
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,//27s
    150,150,150,150,150,150,150,150,150,150,
    30,30,30,30,30,30,30,30,30,30,
    30,30,30,30,30,30,30,30,30,30,
    0,0,0,0,0,0,0,0,0,0
		              
};
uint8_t Yuan_flag=0;

void SysTick_Handler()
{


    num++;
    if(PIT_Flog==0) //�ϵ���ȶ������ǳ�ֵ��
    {
			
        if(num>3000) 
        {
            PIT_Flog=1;
            num=0;
            BEEP=BEEP_OFF;
        }

        if((num>750)&&(num<2250)) 
        { 
            GYRO_VAL=ADC_QuickReadValue(ADC0_SE18_E25);	//��ȡ�����ǳ�ֵ
        }
        
        if((num>2099)&&(num<3000)) 
        { 
            BEEP=BEEP_ON;
        }

    }
    else if(PIT_Flog==1) 
    {  
        //SXT_Timer++;
        if(key==5)
        {
            stopcar_flag = 1;
            LED1 = LED_ON;
        }
        if(stopcar_flag==1)
        {
            StartCarTimer++;
            StopCarJishi++;
            if(StartCarTimer>2000)
            {
                StartCarTimer = 0;
                Sudu_P = sudu_P;
                Sudu_I = sudu_I;
                Sudu_D = sudu_D;
                FX_P = fx_p;
                FX_D = fx_d;
                G_HighestSpeed = g_HighestSpeed;
                Setspeed = setspeed;
            }
            if(StopCarJishi>15000)
            {
                StopCarJishi = 0;
                qipaoxian = 1;
            }

        }
					
        if(num==5) 
        {
            num=0;
            speed_num++;
            Rd_Ad_Value();//��ȡ���ٶȼ���������ADֵ����
            AD_Calculate();//�Ƕ��ںϺ���
            // SXT();
            if(speed_num==20)//100ms
            {
                speed_num=0;
                UpdateSpeed();
            }
            
            if(flag==0)	
            {
                Speed_Calculate(angle_GYRO_y,gyroy);//ֱ���ٶȿ��ƺ���
            }
            else 
            {
                Stopcar();
            }
        }
        if(CarSpeed<(-800)||CarSpeed>3000)
        {
            Stop_Car_Time++;
            if(Stop_Car_Time>=5)
            {
                Stop_Car_Time=0;
                PIT_Flog=0;
                Stopcar();
            }
        }

    }

}

int main(void)
{
    DelayInit();
    DelayMs(10);
    FLASH_Init();          //Ƭ��Flash��ʼ��

#define	D 1
/*
    0----------�����ݴ���Flash��
    1----------��Flash�ж�ȡ����
*/
#if(D==0)  //�����������ת��Ϊ�޷������ʹ���Flash��

    P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
    FLASH_EraseSector(30);		
    FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);	 
    
    D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
    FLASH_EraseSector(31);		
    FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);	 
    
    sudu_P_Flash=(uint32_t)(sudu_P*10000); 
    FLASH_EraseSector(32);		
    FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
		 
    sudu_D_Flash=(uint32_t)(sudu_D*10000); 
    FLASH_EraseSector(33);		
    FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
		 
    sudu_I_Flash=(uint32_t)(sudu_I*10000); 
    FLASH_EraseSector(34);		
    FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
		 
	MMA7361_z_states_Flash=(uint32_t)(MMA7361_z_states); 
	FLASH_EraseSector(35);		
    FLASH_WriteSector(35, (const uint8_t *) (&MMA7361_z_states_Flash), 4, 0);
		 
    setspeed_Flash=(uint32_t)(setspeed); 
    FLASH_EraseSector(36);		
    FLASH_WriteSector(36, (const uint8_t *) (&setspeed_Flash), 4, 0);
		 
    g_HighestSpeed_Flash=(uint32_t)(g_HighestSpeed); 
    FLASH_EraseSector(37);		
    FLASH_WriteSector(37, (const uint8_t *) (&g_HighestSpeed_Flash), 4, 0);
		 
    fx_p_Flash=(uint32_t)(fx_p*10000); 
    FLASH_EraseSector(38);		
    FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);
		 
    fx_d_Flash=(uint32_t)(fx_d*10000); 
    FLASH_EraseSector(39);		
    FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);
								 
#elif (D==1)     //��Flash�е����ݶ���������ת��Ϊ�������������  

/*���õ��ı�����ֵΪ���������澭Flash������ת����Ŀ��õĲ���*/
    P_ANGLE = 0;
    D_ANGLE = 0;
    sudu_P = 0;
    sudu_D = 0;                       
    sudu_I = 0;
    MMA7361_z_states = 0;
    setspeed = 0;
    g_HighestSpeed = 0;
    fx_p = 0;
    fx_d = 0; 
    P_ANGLE_Flash = flash_read(30,0,uint32_t);
    P_ANGLE = ((double)P_ANGLE_Flash/10000);							 
	D_ANGLE_Flash = flash_read(31,0,uint32_t);
    D_ANGLE = ((double)D_ANGLE_Flash/10000);

    sudu_P_Flash = flash_read(32,0,uint32_t);
    sudu_P = ((float)sudu_P_Flash/10000);
								 
    sudu_D_Flash = flash_read(33,0,uint32_t);
    sudu_D = ((float)sudu_D_Flash/10000);

    sudu_I_Flash = flash_read(34,0,uint32_t);
    sudu_I = ((float)sudu_I_Flash/10000);
								 
    MMA7361_z_states_Flash = flash_read(35,0,uint32_t);
    MMA7361_z_states = ((int)MMA7361_z_states_Flash);
								 
    setspeed_Flash = flash_read(36,0,uint32_t);
    setspeed = ((int)setspeed_Flash);
								 
    g_HighestSpeed_Flash = flash_read(37,0,uint32_t);
    g_HighestSpeed = ((int)g_HighestSpeed_Flash);
								 
    fx_p_Flash = flash_read(38,0,uint32_t);
    fx_p = ((float)fx_p_Flash/10000);
								 
    fx_d_Flash = flash_read(39,0,uint32_t);
    fx_d = ((float)fx_d_Flash/10000);
#endif
								 
			
    Tuo_Luo_Yi_Init();  //�����ǳ�ʼ��
    Uart_Init();  //���ڳ�ʼ��
    SXT_7725_Init();
    Beep_Init();  //��������ʼ��
    KEY_Init();  //������ʼ��
    DIP_Init();	 //���뿪�س�ʼ��
    Led_Init();	  //led�Ƴ�ʼ��
    Motor_Init();
    OLED_Init();  //OLED��ʾ��ʼ��
    LCD_Fill(0x00);  //OLED����

    SMQ_Init();//��������ʼ��
    pit_Init();
    SYSTICK_Init(1000);						//�δ�ʱ����ʼ������ʱʱ��Ϊ1ms
    SYSTICK_Cmd(true);
    SYSTICK_ITConfig(true);
				
    while(1)
    { 

    }
}

void Rd_Ad_Value(void)
{
    MMA7361_z = ADC_QuickReadValue(ADC0_SE14_PC0);//���ٶȼ�z��
    MMA7361_x = ADC_QuickReadValue(ADC0_SE8_PB0);//���ٶȼ�x��
    ENC03_y = ADC_QuickReadValue(ADC0_SE18_E25);//������y��v2
    ENC03_x = ADC_QuickReadValue(ADC0_SE17_E24);//������x��v1
}
	
void AD_Calculate(void)
{


    ACC_z=( MMA7361_z-MMA7361_z_states);
    GYRO_y=( ENC03_y-GYRO_VAL);
    anglez_temp[2] = anglez_temp[1];
    anglez_temp[1] = anglez_temp[0];
    anglez_temp[0] =ACC_z; //????z?
    anglez = 0.3333*anglez_temp[2] + 0.3333*anglez_temp[1] + 0.3333*anglez_temp[0];
	gyroy_temp[1] = gyroy_temp[0];
    gyroy_temp[0] = GYRO_y;
    gyroy = 0.42*gyroy_temp[1] + 0.58*gyroy_temp[0];
    anglez = 0.3333*anglez_temp[2] + 0.3333*anglez_temp[1] + 0.3333*anglez_temp[0];
    angle_GYRO_y=KalmanFilter( angle_GYRO_y ,  anglez,  gyroy, 0.005); 			

}

void Speed_Calculate(float angle,float angle_dot)
{
    
    speed_Start = (angle)* P_ANGLE  + angle_dot * D_ANGLE ;
    Speed_L  = Speed_L+speed_Start-SP_out;
	Speed_R	 =Speed_R+speed_Start-SP_out;

    if(Speed_L > Speed_JiYian)
    {
        Speed_L = Speed_JiYian;
    }
    else
    {
        Setspeed = G_HighestSpeed;
    }
    if(Speed_L < (-Speed_JiYian))
    {
        Speed_L = (-Speed_JiYian);
    }
    else
    {
        Setspeed = G_HighestSpeed;
    }
    if(Speed_R > Speed_JiYian)
    {
        Speed_R = Speed_JiYian;
    }
    else
    {
        Setspeed = G_HighestSpeed;
    }
    if(Speed_R < (-Speed_JiYian))
    {
        Speed_R = (-Speed_JiYian);
    }
    else
    {
        Setspeed = G_HighestSpeed;
    }
	if(Dir_ControlPeriod > 4)
    {
        Dir_ControlPeriod = 0;
    }
    DirControlOutput();
    Dir_ControlPeriod++;
    if(SP_ControlPeriod > 20)
    {
        SP_ControlPeriod = 0;
    }
    SPControlOutput();
    SP_ControlPeriod++;
	
    Go_Speed(Speed_L-Dir_OUT-WheelError,Speed_R+Dir_OUT+WheelError);
						//	Go_Speed(Speed_L,Speed_R);

//Go_Speed(Speed_L,Speed_R);
}
void Go_Speed(int Speed_L,int	Speed_R)
{
/////////////������ȥ��ѹ///////////////////////////
    if(Speed_L > 0)
    {
        Speed_L += MOTOR_DEAD_VAL_L_;
    }
    else 
    {
        Speed_L -= MOTOR_DEAD_VAL_L_;
    }
    if(Speed_R>0)
    {
        Speed_R+=MOTOR_DEAD_VAL_R_;
    }
    else
    {
        Speed_R-=MOTOR_DEAD_VAL_R_;
    }
	////////////////////����///////////////////////////////////
    if(Speed_L < (-Speed_JiYian))
    {
        Speed_L = -Speed_JiYian;
    }
    if(Speed_L > Speed_JiYian) 
    {
        Speed_L = Speed_JiYian;
    }
    if(Speed_R < (-Speed_JiYian)) 
    {
        Speed_R = -Speed_JiYian;
    }
    if(Speed_R > Speed_JiYian) 
    {
        Speed_R = Speed_JiYian; 
    }

    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH2, 5000+Speed_L ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH0, 5000-Speed_L ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH1, 5000+Speed_R ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH3, 5000-Speed_R ); 

}


void KEY_Init(void)            //������ʼ��
{
    GPIO_QuickInit(HW_GPIOB, 20, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOB, 23, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOC, 5,  kGPIO_Mode_IPU);
}
void Key_Scan(void)           //����״̬ɨ�躯��
{ 
    u8 i=100;
	if(KEY1==KEY_SINGLE||KEY2==KEY_SINGLE||KEY3==KEY_SINGLE||KEY4==KEY_SINGLE||KEY5==KEY_SINGLE)
    {
        while(i--);
        if(KEY1==KEY_SINGLE)
        {
            key=1;
        }
        else if(KEY2==KEY_SINGLE)
        {
            key=2;
        }
        else if(KEY3==KEY_SINGLE)
        {
            key=3;
        }
        else if(KEY4==KEY_SINGLE)
        {
            key=4;
        }
        else if(KEY5==KEY_SINGLE)
        {
            key=5;
        }
	}
}

void DIP_Init(void)					//���뿪�س�ʼ��
{
    GPIO_QuickInit(HW_GPIOA,  4, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOA,  5, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOA, 12, kGPIO_Mode_IPU);
    GPIO_QuickInit(HW_GPIOA, 13, kGPIO_Mode_IPU);
}
void Dip_Scan(void)					//���뿪��״̬ɨ�躯��
{
    if(DIP1==DIP_SINGLE)
    {
        dip1=ON;
    }
    else 
    {
        dip1=OFF;
    }
    if(DIP2==DIP_SINGLE)
    {
        dip2=ON;
    }
    else
    {
        dip2=OFF;
    }
    if(DIP3==DIP_SINGLE)
    {
        dip3=ON;
    }
    else
    {
        dip3=OFF;
    }
    if(DIP4==DIP_SINGLE)
    {
        dip4=ON;
    }
    else
    {
        dip4=OFF;
    }
}
void Led_Init(void)						//led�Ƴ�ʼ��
{
    GPIO_QuickInit(HW_GPIOA,  14, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOA,  15, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOA,  16, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOA,  17, kGPIO_Mode_OPP);
    LED1=LED_OFF;
    LED2=LED_OFF;
    LED3=LED_OFF;
    LED4=LED_OFF;
	  
}
void Tuo_Luo_Yi_Init(void)			//�����ǳ�ʼ��
{
    ADC_QuickInit(ADC0_SE8_PB0,kADC_SingleDiff12or13);
    ADC_QuickInit(ADC0_SE9_PB1,kADC_SingleDiff12or13);
    ADC_QuickInit(ADC0_SE14_PC0,kADC_SingleDiff12or13);
    ADC_QuickInit(ADC0_SE17_E24,kADC_SingleDiff12or13);
    ADC_QuickInit(ADC0_SE18_E25,kADC_SingleDiff12or13);
    PORT_PinMuxConfig(HW_GPIOB,0,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOB,1,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOC,0,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOE,24,kPinAlt0);
    PORT_PinMuxConfig(HW_GPIOE,25,kPinAlt0);
    ADC_StartConversion(HW_ADC0, 8, kADC_MuxA);
    ADC_StartConversion(HW_ADC0, 9, kADC_MuxA);
    ADC_StartConversion(HW_ADC0, 14, kADC_MuxA);
    ADC_StartConversion(HW_ADC0, 17, kADC_MuxA);
    ADC_StartConversion(HW_ADC0, 18, kADC_MuxA);
}


void Motor_Init(void)				//�����ʼ����Ĭ��ռ�ձ�Ϊ50%
{
    FTM_PWM_QuickInit(FTM0_CH0_PC01, kPWM_EdgeAligned, 20000);//c1���ֺ�
    FTM_PWM_QuickInit(FTM0_CH1_PC02, kPWM_EdgeAligned, 20000);//c2����ǰ��
    FTM_PWM_QuickInit(FTM0_CH2_PC03, kPWM_EdgeAligned, 20000);//c3���ֺ�
    FTM_PWM_QuickInit(FTM0_CH3_PC04, kPWM_EdgeAligned, 20000);//c4����ǰ��
}


void Beep_Init(void)					//��������ʼ��
{
    GPIO_QuickInit(HW_GPIOC,  6, kGPIO_Mode_OPP);
}


void Uart_Init(void)					//���ڳ�ʼ��
{	
    UART_QuickInit(UART3_RX_PB10_TX_PB11, 9600);//��ʼ������
    /*  ����UART �ж����� �򿪽����ж� ��װ�жϻص����� */
    UART_CallbackRxInstall(HW_UART3, UART_RX_ISR);
    /* �򿪴��ڽ����жϹ��� IT �����жϵ���˼*/
    UART_ITDMAConfig(HW_UART3, kUART_IT_Rx, true);
}

static void UART_RX_ISR(uint16_t byteReceived)//�����жϺ���
{

    UART_WriteByte(HW_UART3, byteReceived);
    if(byteReceived==0x0035)
    {
        flag=!flag;
        GPIO_ToggleBit(HW_GPIOA, 15);
    }
}

static void PIT0_CallBack(void)//pit�жϷ�����
{
    key_flag++;
    SXT_Timer++;
    tuxiang_Timer++;
    if(key_flag>=183)//183����ִ��һ�ΰ���ɨ��
    {
        key_flag=0;
        Key_Scan();//����ɨ�躯��
        Dip_Scan();//���뿪��ɨ�躯��
        Key();//��������
        Oled_show();//��ʾ����
		
    }
    
    if(SXT_Timer==20)
    {
        SXT_Timer=0;     
        SXT();
    }
    
    if(tuxiang_Timer==10)
    {
        tuxiang_Timer=0;
        Tu_Xiang();
    }
}

static void GPIO_ISR(uint32_t array)//���������жϺ���
{

    if(array & (1 << 17))
    {
        if(SQM_R==1)
        {
            suduR++;
        }
        else
        {
            suduR--;
        }
        PORTB->ISFR |= (1<<17);
		
    }
    if(array & (1 << 18))
	{
        if(SQM_L==1)
        {
            suduL++;
        }
        else
        {
            suduL--;
        }
        PORTB->ISFR |= (1<<18);
		
    }

}
	
void Key(void)
{
    if(key==1)
    {
        key=0;
        LCD_Fill(0x00);
        k5_flag++;
        k5_flag_ = 0;
        if(k5_flag >= 10)
        {
            k5_flag = 0;
        }

    }

    if(key==2)
    {
        key=0;
        LCD_Fill(0x00);
        k5_flag_++;
        if(k5_flag_ >= 4)
        {
            k5_flag_=0;
        }

    }
    if(key==3)
    {
        key=0;
        switch(k5_flag)
        {
            case 0:  
                {
                    switch(k5_flag_)
                    {
                        case 0:
                            P_ANGLE+=1; 
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);
                            break; 
                        case 1:
                            P_ANGLE+=0.1; 
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0); 
                            break;
                        case 2:	
                            P_ANGLE+=0.001;
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0); 
                            break; 
                        case 3:
                            P_ANGLE+=0.0001; 
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0); 
                            break; 
                    }
                }break;
	        case 1:   
                {
                    switch(k5_flag_)
                    {
                        case 0:
                            D_ANGLE+=1;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;	
                        case 1:
                            D_ANGLE+=0.1;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;					
                        case 2:
                            D_ANGLE+=0.001;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;
                        case 3:
                            D_ANGLE+=0.0001;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;
                    }
                }break;
            case 2:  
                {
                    switch(k5_flag_)
                    {
                        case 0:	
                            sudu_P+=1;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 1:	
                            sudu_P+=0.1;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 2:	
                            sudu_P+=0.001;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 3:	
                            sudu_P+=0.0001;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;

                    }
                }break;	
            case 3:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            sudu_D+=1;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;					
                        case 1:	
                            sudu_D+=0.1;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;			
                        case 2:
                            sudu_D+=0.001;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;					
                        case 3:	
                            sudu_D+=0.0001;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;					
                    }
                }break;
            case 4:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            sudu_I+=1;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;	
                        case 1: 
                            sudu_I+=0.1;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                        case 2: 
                            sudu_I+=0.001;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                        case 3: 
                            sudu_I+=0.0001;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                    }
                }break;				 
            case 5:	
                {
                    switch(k5_flag_)
                    {
                        case 0:
                            MMA7361_z_states+=1;
                            MMA7361_z_states_Flash=(uint32_t)MMA7361_z_states;
                            FLASH_EraseSector(35);
                            FLASH_WriteSector(35, (const uint8_t *) (&MMA7361_z_states_Flash), 4, 0);
                            break;								
                        case 1: break;
                        case 2: break; 
                        case 3: break; 
                    }
                }break;
            case 6:	
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            setspeed+=10;g_HighestSpeed+=10;
                            setspeed_Flash=(uint32_t)setspeed;
                            FLASH_EraseSector(36);
                            FLASH_WriteSector(36, (const uint8_t *) (&setspeed_Flash), 4, 0);
                            g_HighestSpeed_Flash=(uint32_t)g_HighestSpeed;
                            FLASH_EraseSector(37);
                            FLASH_WriteSector(37, (const uint8_t *) (&g_HighestSpeed_Flash), 4, 0);					
                            break;
                        case 1: break;
                        case 2: break; 
                        case 3: break; 
                    }
                }break;	
            case 7:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            fx_p+=1;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;					
                        case 1: 
                            fx_p+=0.1;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;	
                        case 2: 
                            fx_p+=0.001;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;	
                        case 3:
                            fx_p+=0.0001;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;					
                    }
                }break;
            case 8:
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            fx_d+=1;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;	
                        case 1: 
                            fx_d+=0.1;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;
                        case 2:
                            fx_d+=0.001;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;
                        case 3:
                            fx_d+=0.0001;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;					
                    }
                }break;					 
            case 9:break;			
        }
    }
    if(key==4)
    {
        key=0;
        switch(k5_flag)
        {
            case 0:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            P_ANGLE-=1;
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);
                            break; 					
                        case 1: 
                            P_ANGLE-=0.1;
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);
                            break; 
                        case 2: 
                            P_ANGLE-=0.001;
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);
                            break; 
                        case 3: 
                            P_ANGLE-=0.0001;
                            P_ANGLE_Flash=(uint32_t)(P_ANGLE*10000); 
                            FLASH_EraseSector(30);		
                            FLASH_WriteSector(30, (const uint8_t *) (&P_ANGLE_Flash), 4, 0);
                            break; 					
                    }
                }break;
            case 1:   
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            D_ANGLE-=1;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;
                        case 1: 
                            D_ANGLE-=0.1;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                                break;
                        case 2:
                            D_ANGLE-=0.001;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;					
                        case 3:
                            D_ANGLE-=0.0001;
                            D_ANGLE_Flash=(uint32_t)(D_ANGLE*10000); 
                            FLASH_EraseSector(31);		
                            FLASH_WriteSector(31, (const uint8_t *) (&D_ANGLE_Flash), 4, 0);
                            break;
                    }
                }break;
            case 2:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            sudu_P-=1;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 1: 
                            sudu_P-=0.1;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 2: 
                            sudu_P-=0.001;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                        case 3: 
                            sudu_P-=0.0001;
                            sudu_P_Flash=(uint32_t)(sudu_P*10000);
                            FLASH_EraseSector(32);
                            FLASH_WriteSector(32, (const uint8_t *) (&sudu_P_Flash), 4, 0);
                            break;
                    }
                }break;
            case 3:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            sudu_D-=1;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;	
                        case 1: 
                            sudu_D-=0.1;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;	
                        case 2: 
                            sudu_D-=0.001;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;	
                        case 3: 
                            sudu_D-=0.0001;
                            sudu_D_Flash=(uint32_t)(sudu_D*10000);
                            FLASH_EraseSector(33);
                            FLASH_WriteSector(33, (const uint8_t *) (&sudu_D_Flash), 4, 0);
                            break;	
                    }
                }break;	
            case 4:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            sudu_I-=1;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                        case 1: 
                            sudu_I-=0.1;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                        case 2:
                            sudu_I-=0.001;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                        case 3: 
                            sudu_I-=0.0001;
                            sudu_I_Flash=(uint32_t)(sudu_I*10000);
                            FLASH_EraseSector(34);
                            FLASH_WriteSector(34, (const uint8_t *) (&sudu_I_Flash), 4, 0);
                            break;
                    }
                }break;				 
     
            case 5:	
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            MMA7361_z_states-=1;
                            MMA7361_z_states_Flash=(uint32_t)MMA7361_z_states;
                            FLASH_EraseSector(35);
                            FLASH_WriteSector(35, (const uint8_t *) (&MMA7361_z_states_Flash), 4, 0);
                            break;
                        case 1:break;
                        case 2:break; 
                        case 3:break; 
                    }
                }break;	
            case 6:	
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            setspeed-=10;g_HighestSpeed-=10;
                            setspeed_Flash=(uint32_t)setspeed;
                            FLASH_EraseSector(36);
                            FLASH_WriteSector(36, (const uint8_t *) (&setspeed_Flash), 4, 0);
                            g_HighestSpeed_Flash=(uint32_t)g_HighestSpeed;
                            FLASH_EraseSector(37);
                            FLASH_WriteSector(37, (const uint8_t *) (&g_HighestSpeed_Flash), 4, 0);					
                            break;
                        case 1:break;
                        case 2:break; 
                        case 3:break; 
                    }
                }break;		
            case 7:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            fx_p-=1;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;					
                        case 1: 
                            fx_p-=0.1;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;
                        case 2: 
                            fx_p-=0.001;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;
                        case 3: 
                            fx_p-=0.0001;
                            fx_p_Flash=(uint32_t)(fx_p*10000);
                            FLASH_EraseSector(38);
                            FLASH_WriteSector(38, (const uint8_t *) (&fx_p_Flash), 4, 0);					
                            break;
                    }
                }break;		
            case 8:  
                {
                    switch(k5_flag_)
                    {
                        case 0: 
                            fx_d-=1;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;	
                        case 1:
                            fx_d-=0.1;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;	
                        case 2: 
                            fx_d-=0.001;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;	
                        case 3: 
                            fx_d-=0.0001;
                            fx_d_Flash=(uint32_t)(fx_d*10000);
                            FLASH_EraseSector(39);
                            FLASH_WriteSector(39, (const uint8_t *) (&fx_d_Flash), 4, 0);					
                            break;	
                    }
                }break;				 
            case 9:break;			
        }
    }


}

void Oled_show(void)
{
    switch(k5_flag)
    {
        case 0://�Ƕ�p
            {
                switch(k5_flag_)
                {
                    case 0:
                        {
                            LCD_Print(50, 0,"P1",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                            LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                            OLED_ShowNum(25,55,D_ANGLE,9,16);	
                            OLED_ShowNum(89,23,Speed_R,4,16);	
                            OLED_ShowNum(25,39,P_ANGLE,9,16); 
                        }break;
                    case 1:
                        {
                            LCD_Print(45, 0,"P0.1",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                            LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                            OLED_ShowNum(25,55,D_ANGLE*10,9,16);	
                            OLED_ShowNum(89,23,Speed_R,4,16);	
                            OLED_ShowNum(25,39,P_ANGLE*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(40, 0,"P0.001",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                            LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                            OLED_ShowNum(25,55,D_ANGLE*1000,9,16);	
                            OLED_ShowNum(89,23,Speed_R,4,16);	
                            OLED_ShowNum(25,39,P_ANGLE*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(35, 0,"P0.0001",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                            LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                            OLED_ShowNum(25,55,D_ANGLE*10000,9,16);	
                            OLED_ShowNum(89,23,Speed_R,4,16);	
                            OLED_ShowNum(25,39,P_ANGLE*10000,9,16); 
                        }break;
                	
                }
            		

            }break;
        case 1://�Ƕ�d
            {
                switch(k5_flag_)
                {
                case 0:
                    {
                        LCD_Print(50, 0,"D1",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                        LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                        OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                        OLED_ShowNum(25,55,D_ANGLE,9,16);	
                        OLED_ShowNum(89,23,Speed_R,4,16);	
                        OLED_ShowNum(25,39,P_ANGLE,9,16); 
                    }break;
                case 1:
                    {
                        LCD_Print(45, 0,"D0.1",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                        LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                        OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                        OLED_ShowNum(25,55,D_ANGLE*10,9,16);	
                        OLED_ShowNum(89,23,Speed_R,4,16);	
                        OLED_ShowNum(25,39,P_ANGLE*10,9,16); 
                    }break;
                case 2:
                    {
                        LCD_Print(40, 0,"D0.001",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                        LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                        OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                        OLED_ShowNum(25,55,D_ANGLE*1000,9,16);	
                        OLED_ShowNum(89,23,Speed_R,4,16);	
                        OLED_ShowNum(25,39,P_ANGLE*1000,9,16); 
                    }break;
                case 3:
                    {
                        LCD_Print(35, 0,"D0.0001",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 16,"r :",TYPE16X16,TYPE8X16);
                        LCD_Print(64,16,"S :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 32,"P :",TYPE16X16,TYPE8X16);
                        OLED_ShowNum(25,23,angle_GYRO_y,4,16);
                        OLED_ShowNum(25,55,D_ANGLE*10000,9,16);	
                        OLED_ShowNum(89,23,Speed_R,4,16);	
                        OLED_ShowNum(25,39,P_ANGLE*10000,9,16); 
                    }break;

                }
            		

            }break;
        case 2://�ٶ�p
            {
                switch(k5_flag_)
                {
                    case 0:
                        {
                            LCD_Print(40, 0,"sud_P1",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"P :",TYPE16X16,TYPE8X16);				      
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);					     
                            OLED_ShowNum(25,55,sudu_D,9,16);	
                            OLED_ShowNum(25,39,sudu_I,9,16);	
                            OLED_ShowNum(25,23,sudu_P,9,16); 
                        }break;
                    case 1:
                        {
                            LCD_Print(35, 0,"sud_P0.1",TYPE16X16,TYPE8X16);		
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);					    
                            OLED_ShowNum(25,55,sudu_D*10,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(30, 0,"sud_P0.001",TYPE16X16,TYPE8X16);					   
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);				   
                            OLED_ShowNum(25,55,sudu_D*1000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*1000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(25, 0,"sud_P0.0001",TYPE16X16,TYPE8X16);					   
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);				    
                            OLED_ShowNum(25,55,sudu_D*10000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10000,9,16); 
                        }break;

                }
            		

            }break;	
        case 3://�ٶ�d
            {
                switch(k5_flag_)
                {
                    case 0:
                    {
                        LCD_Print(40, 0,"sud_D1",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 16,"P :",TYPE16X16,TYPE8X16);			      
                        LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                        LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);				   
                        OLED_ShowNum(25,55,sudu_D,9,16);	
                        OLED_ShowNum(25,39,sudu_I,9,16);	
                        OLED_ShowNum(25,23,sudu_P,9,16); 
                    }break;
                    case 1:
                        {
                            LCD_Print(35, 0,"sud_D0.1",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*10,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(30, 0,"sud_D0.001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*1000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*1000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(25, 0,"sud_D0.0001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*10000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10000,9,16); 
                        }break;

                }
            		

            }break;
        case 4://�ٶ�i
            {
                switch(k5_flag_)
                {
                    case 0:
                        {
                            LCD_Print(40, 0,"sud_I1",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D:",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D,9,16);	
                            OLED_ShowNum(25,39,sudu_I,9,16);	
                            OLED_ShowNum(25,23,sudu_P,9,16); 
                        }break;
                    case 1:
                        {
                            LCD_Print(35, 0,"sud_I0.1",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D:",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*10,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(30, 0,"sud_I0.001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D:",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*1000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*1000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(25, 0,"sud_I0.0001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"I :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 48,"D:",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,55,sudu_D*10000,9,16);	
                            OLED_ShowNum(25,39,sudu_I*10000,9,16);	
                            OLED_ShowNum(25,23,sudu_P*10000,9,16); 
                        }break;

                }
            		

            }break;						 
        case 5:	
            {
                LCD_Print(0, 0,"MMA7361_z_states",TYPE16X16,TYPE8X16);
                LCD_Print(0,32,"z :",TYPE16X16,TYPE8X16);
                OLED_ShowNum(25,39,MMA7361_z_states,9,16);
            
            }break;
        case 6:	
            {
                LCD_Print(30, 0,"setSpeed",TYPE16X16,TYPE8X16);
                LCD_Print(0,32,"s :",TYPE16X16,TYPE8X16);
                OLED_ShowNum(25,39,setspeed,9,16);
            }break;
        case 7://����p
            {
                switch(k5_flag_)
                {
                    case 0:
                        {
                            LCD_Print(40, 0,"fx_p",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,39,fx_d,9,16);	
                            OLED_ShowNum(25,23,fx_p,9,16); 
                        }break;
                    case 1:
                        {
                            LCD_Print(35, 0,"fx_p0.1",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);	
                            OLED_ShowNum(25,39,fx_d*10,9,16);	
                            OLED_ShowNum(25,23,fx_p*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(30, 0,"fx_p0.001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,39,fx_d*1000,9,16);	
                            OLED_ShowNum(25,23,fx_p*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(25, 0,"fx_p0.0001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);	
                            OLED_ShowNum(25,39,fx_d*10000,9,16);	
                            OLED_ShowNum(25,23,fx_p*10000,9,16); 
                        }break;

                }
            		

            }break;						 			
        case 8://����p
            {
                switch(k5_flag_)
                {
                    case 0:
                        {
                            LCD_Print(40, 0,"fx_d",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,39,fx_d,9,16);	
                            OLED_ShowNum(25,23,fx_p,9,16); 
                        }break;
                    case 1:
                        {
                            LCD_Print(35, 0,"fx_d0.1",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);	
                            OLED_ShowNum(25,39,fx_d*10,9,16);	
                            OLED_ShowNum(25,23,fx_p*10,9,16); 
                        }break;
                    case 2:
                        {
                            LCD_Print(30, 0,"fx_d0.001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);
                            OLED_ShowNum(25,39,fx_d*1000,9,16);	
                            OLED_ShowNum(25,23,fx_p*1000,9,16); 
                        }break;
                    case 3:
                        {
                            LCD_Print(25, 0,"fx_d0.0001",TYPE16X16,TYPE8X16);
                            LCD_Print(0,16,"P :",TYPE16X16,TYPE8X16);
                            LCD_Print(0, 32,"D :",TYPE16X16,TYPE8X16);	
                            OLED_ShowNum(25,39,fx_d*10000,9,16);	
                            OLED_ShowNum(25,23,fx_p*10000,9,16); 
                        }break;

                }


            }break;						 							 
    		
        case 9:	
            dis_bmp(60,80,image_dec[0],0x01);break; 
    }


}
void SMQ_Init(void)//��������ʼ��
{
    GPIO_CallbackInstall(HW_GPIOB, GPIO_ISR);
    GPIO_QuickInit(HW_GPIOB, 16, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOB, 16, kGPIO_IT_FallingEdge, true);
    GPIO_QuickInit(HW_GPIOB, 17, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOB, 17, kGPIO_IT_FallingEdge, true);
    GPIO_QuickInit(HW_GPIOB, 18, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOB, 18, kGPIO_IT_FallingEdge, true);
    GPIO_QuickInit(HW_GPIOB, 19, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOB, 19, kGPIO_IT_FallingEdge, true);
	
}

void UpdateSpeed(void)
{
	
    CarSpeed = (suduL + suduR)/2;
    WheelError = suduL - suduR;  //�����ٶȲ�
    suduL = suduR = 0;

    offset2 = LastError;
    LastError = SpeedError;
    SpeedError = Setspeed - CarSpeed;  //���ε�ƫ��

    Speed_out =  (Sudu_P * SpeedError + Sudu_I * LastError + Sudu_D * offset2);
    SP_out_old=SP_out_new;
    SP_out_new=Speed_out;
	
	
}

void OV_ISR(uint32_t index)
{

    /* ���ж� */
    if(index & (1 << BOARD_OV7725_VSYNC_PIN))
    {
        switch(status)
        {
            case kIdle:     /* ��ʼ��׽*/
                DMA_SetMajorLoopCounter(BOARD_OV7725_DMA, OV7725_SIZE);
                DMA_SetDestAddress(BOARD_OV7725_DMA, (uint32_t)gImageBuffer);  //Ŀ���ַ
                DMA_EnableRequest(BOARD_OV7725_DMA); 
                status = kInCapture;
                break;
            case kInCapture:  /*��׽��ϣ��ر��ж�*/
                GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, false);
                //PORTC->ISFR = 0xFFFFFFFF;  //����жϱ�־λ
                PORTC->ISFR|=(1<<16);
                status = kComplete;
                break;
        }
    }
}

void Tu_Xiang(void)
{
    if(status == kComplete)
    {

        Image_Decompression(gImageBuffer,image_dec[0]);  //��ѹ
        Find_Mid_Line(image_dec);


        //show_60x80_7725(gImageBuffer);                 //OLED��ʾ
        /* �����жϣ���ʼ��һ�β�׽ */
        status = kIdle;
        GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, true);
    }
				
					
}

void Find_Mid_Line(u8  image_dec[OV7725_H][OV7725_W])//Ѱ����
{
    uint8_t guaidian_xielu=0;
    uint8_t L_barrier_black1=0;         //????    
    uint8_t L_barrier_black2=0;       //????
    uint8_t L_find_write_black=0;
    uint8_t L_find_black_write=0;
    uint8_t R_barrier_black1=0;         //????    
    uint8_t R_barrier_black2=0;       //????
    uint8_t R_find_write_black=0;
    uint8_t R_find_black_write=0;
     uint8_t guai_dian=0;
    uint8_t L_barrier=0,R_barrier=0;

    quanzhi_sum=0;

    mid_line_sum = 0;
    mid_line_mean = 0;

    int  row2=0,line2=0; //????????
    unsigned char Left_RF=0,Right_LF=0,Left_F=0,Right_F=0,yuan_l_f=0,yuan_r_f=0;
    int row3=0,line3=0;
    int row4=0,line4=0;
    int row=0,line=0;
	
	///////////////////��ǰ6�м򵥵��˲�///////////////////////////////////
    for(row2=ROW_MAX-1; row2>=ROW_MAX-6; row2--)
    { 
        if(row2==63)//64???
        {     
            for(line2=1;line2<LINE_MAX-2;line2++)
            { 
                if(image_dec[63][line2]==1)
                {
                    if((image_dec[row2][line2-1]==0)&&(image_dec[row2][line2+1]==0))    //?????????????,???
                    image_dec[row2][line2]=0;  
                } 

            }
        }
        else//63??57???
        {   
            for(line2=1;line2<LINE_MAX-2;line2++)
            { 
                if(image_dec[row2][line2]==1)
                {
                    if((image_dec[row2][line2-1]==0)&&(image_dec[row2][line2+1]==0))    //?????????????,???
                    {
                        image_dec[row2][line2]=0;
                    }
                    else if((image_dec[row2-1][line2]==0)&&(image_dec[row2+1][line2]==0))    //?????????????,???
                    {
                        image_dec[row2][line2]=0; 
                    }
                } 

            }
        }
    }

/////////////////////////ǰ6���м�������Ѳ�ߣ���Ϊ����Ļ�׼��/////////////////////////////
    for(row2=ROW_MAX-1; row2>=ROW_MAX-6; row2--)//??64~59?/[63]~[58]   
    {   
        if(qipaoxian==1)//������ͣ��
        {
            Qipaoxian();
        }
        for(line2=40;line2>=0;line2--)  //���м�����Ѳ��
        {
            if((char)image_dec[row2][line2+1]-(char)image_dec[row2][line2]==(1))  //Ѱ���׵��������
            {

                if(line2==0)
                {
                    EdgeLeft[row2]=line2;  
                    EdgeLeftceshi[row2]=line2;  
                    Left_F=1;
                    break; 
                }
                else if((image_dec[row2][line2-1]==0)&&(image_dec[row2][line2+2]==1)) 
                {
                    EdgeLeft[row2]=line2;  
                    EdgeLeftceshi[row2]=line2;  
                    Left_F=1;
                    break; 
                            
                } 

            } 
        }
        if(Left_F==0)//��߶���
        {  
            if(row2==59) 
            {
                EdgeLeft[row2]=0;
            }
            else  
            {
                EdgeLeft[row2]=EdgeLeft[row2+1];
                EdgeLeftceshi[row2]=LL;
            }
        } 
      
    // Left_F=0;//?????
         
    //���м�����Ѳ��         
        for(line2=41;line2<=LINE_MAX-1;line2++) 
        {
            if((char)image_dec[row2][line2-1]-(char)image_dec[row2][line2]==(1))  //???????????
            {
                if(line2==79)
                {
                    EdgeRight[row2]=line2;  //?????????
                    EdgeRightceshi[row2]=line2;
                    Right_F=1;
                    break;                   
                }
                else if((image_dec[row2][line2-2]==1)&&(image_dec[row2][line2+1]==0)) 
                {
                    EdgeRight[row2]=line2;  //?????????
                    EdgeRightceshi[row2]=line2;
                    Right_F=1;
                    break;                  
                }
            }
          
        }
        if(Right_F==0)//�ұ߶���
        {    
            if(row2==59) 
            {
                EdgeRight[row2]=79;
            }
            else
            {
                 EdgeRight[row2]=EdgeRight[row2+1];
                 EdgeRightceshi[row2]=RL;
            }
        }

        centerline[row2]=(EdgeRight[row2]+EdgeLeft[row2])/2;
        centerlineceshi[row2]=(EdgeRightceshi[row2]+EdgeLeftceshi[row2])/2;

    }

       

/******************************************6????????********************************************/ 
  

    for(row3=ROW_MAX-7;row3>=0;row3--)
    {
        //////////////Բ�������ж�///////////////////  

        if(H<30)
        {
            if(image_dec[row3][39]==0)
            {
                if(image_dec[row3-1][39]==image_dec[row3-2][39]&&
                    image_dec[row3-1][39]==0)
                {
                    if((image_dec[row3][10]==image_dec[row3][70])&&
                    	(image_dec[row3-1][10]==image_dec[row3-1][70])
                        &&image_dec[row3][10]==1)
                    {
                    yuan_jishu++;
                    	if(yuan_jishu==3)
                    		{
                    	yuan_flag=1;
                    	yuan_jishu=0;
                    	  }
                    }
                }
            }
        }
    //////////////Բ�������ж�///////////////////  
        if(EdgeLeft[row3+1]==0)//��һ����߽�Ϊ����
        {  
              
            for(line3=EdgeLeft[row3+1];line3<=EdgeLeft[row3+1]+5;line3++)  //����һ����߽���������5�����ص�
            {
              
                if((char)image_dec[row3][line3+1]-(char)image_dec[row3][line3]==(1))//�ҵ���߽�
                {
                    EdgeLeft[row3]=line3;
                    EdgeLeftceshi[row3]=line3;
                    Left_F=1; 
                    break;
                }
            }
        }
        else if(EdgeLeft[row3+1]==1)//��һ����߽�Ϊ1
        {  
              
            for(line3=EdgeLeft[row3+1];line3<=EdgeLeft[row3+1]+5;line3++)  //
            {
              
                if((char)image_dec[row3][line3+1]-(char)image_dec[row3][line3]==(1))//
                {
                    EdgeLeft[row3]=line3;
                    EdgeLeftceshi[row3]=line3;
                    Left_F=1;
                    Left_RF=1; 
                    break;
                }

            }
               
            if(Left_RF==0)//����߽�����û���ѵ�߽�
            {
                if(image_dec[row3][0]==0)
                {
                    EdgeLeft[row3]=0;
                    EdgeLeftceshi[row3]=EdgeLeft[row3];
                    Left_F=1;
                }
            }
         
        } 
        else if(EdgeLeft[row3+1]==2)//��߽�Ϊ2
        {  
              
            for(line3=EdgeLeft[row3+1];line3<=EdgeLeft[row3+1]+5;line3++)  
            {
              
                if((char)image_dec[row3][line3+1]-(char)image_dec[row3][line3]==(1))//Ѱ����߽�
                {
                    EdgeLeft[row3]=line3;
                    EdgeLeftceshi[row3]=line3;
                    Left_F=1;
                    Left_RF=1; 
                    break;
                }
            }

            if(Left_RF==0)//����߽���һ������ûѰ��߽�
            {  
                if(image_dec[row3][1]==0)
                {
                    EdgeLeft[row3]=1;
                    EdgeLeftceshi[row3]=EdgeLeft[row3];
                    Left_F=1;
                }
                else if(image_dec[row3][0]==0)
                {
                    EdgeLeft[row3]=0;
                    EdgeLeftceshi[row3]=EdgeLeft[row3];
                    Left_F=1;
                }
            }
        }
        else
        {  
            for(line3=EdgeLeft[row3+1];line3<=EdgeLeft[row3+1]+5;line3++)  //��һ����߽�Ȳ���0,1,2
            {
              
                if((char)image_dec[row3][line3+1]-(char)image_dec[row3][line3]==(1))//??????
                {
                    EdgeLeft[row3]=line3;
                    EdgeLeftceshi[row3]=line3;
                    Left_F=1;
                    Left_RF=1; 
                    break;
                }
             
            }
              
                
            if(Left_RF==0)
            {  
                for(line3=EdgeLeft[row3+1];line3>=EdgeLeft[row3+1]-5;line3--)  //???????????,??????????
                {
                    if((char)image_dec[row3][line3]-(char)image_dec[row3][line3-1]==(1))//??????
                    {
                        EdgeLeft[row3]=line3-1;
                        EdgeLeftceshi[row3]=line3-1;
                        Left_F=1;
                        break;
                    } 
                }
            }
        }

        if( Left_F==0)//��߶���
        { 
            EdgeLeft[row3]=EdgeLeft[row3+1];
            EdgeLeftceshi[row3]=LL; 
        }
    //    Left_F=0;
    //    Left_RF=0;
         
    /////////////////////////////////��Ѱ�ұ߽�//////////////////////
       
        if(EdgeRight[row3+1]==79)
        {  
              
            for(line3=EdgeRight[row3+1];line3>=EdgeRight[row3+1]-5;line3--)  
            { 
               
                if((char)image_dec[row3][line3-1]-(char)image_dec[row3][line3]==(1))
                {
                    EdgeRight[row3]=line3;
                    EdgeRightceshi[row3]=line3; 
                    Right_F=1;
                    break;
                } 
             
            }
             
        }
        else if(EdgeRight[row3+1]==78)
        {  
               
              
            for(line3=EdgeRight[row3+1];line3>=EdgeRight[row3+1]-5;line3--)  //???????????,??????????
            { 
              
                if((char)image_dec[row3][line3-1]-(char)image_dec[row3][line3]==(1))//??????
                {
                    EdgeRight[row3]=line3;
                    EdgeRightceshi[row3]=line3; 
                    Right_F=1;
                    Right_LF=1;
                    break;
                } 
            }
               
            if(Right_LF==0)  
            {
                if(image_dec[row3][79]==0)
                {
                    EdgeRight[row3]=79;
                    //  EdgeRightceshi[row3]=line3+1;
                    EdgeRightceshi[row3]=EdgeRight[row3];


                    Right_F=1; 
                }
            }
             
        }
        else if(EdgeRight[row3+1]==77)
        {  
             
            for(line3=EdgeRight[row3+1];line3>=EdgeRight[row3+1]-5;line3--)  //???????????,??????????
            { 
               
                if((char)image_dec[row3][line3-1]-(char)image_dec[row3][line3]==(1))//??????
                {
                    EdgeRight[row3]=line3;
                    EdgeRightceshi[row3]=line3; 
                    Right_F=1;
                    Right_LF=1;
                    break;
                } 
            }
              
            if(Right_LF==0)  
            {

                if(image_dec[row3][78]==0)
                {
                    EdgeRight[row3]=78;
                    //EdgeRightceshi[row3]=line3+1;
                    EdgeRightceshi[row3]=EdgeRight[row3];


                    Right_F=1; 
                }
                if(image_dec[row3][79]==0)
                {
                    EdgeRight[row3]=79;
                    // EdgeRightceshi[row3]=line3+2;
                    EdgeRightceshi[row3]=EdgeRight[row3];
                    Right_F=1; 
                }
            }
               
        }
        else
        {  
              
            for(line3=EdgeRight[row3+1];line3>=EdgeRight[row3+1]-5;line3--)  //???????????,??????????
            { 

                if((char)image_dec[row3][line3-1]-(char)image_dec[row3][line3]==(1))//??????
                {
                    EdgeRight[row3]=line3;
                    EdgeRightceshi[row3]=line3; 
                    Right_F=1;
                    Right_LF=1;
                    break;
                }

            }
               
            if(Right_LF==0)  
            {
                for(line3=EdgeRight[row3+1];line3<=EdgeRight[row3+1]+5;line3++)  //???????????,??????????
                { 
                    if((char)image_dec[row3][line3]-(char)image_dec[row3][line3+1]==(1))//??????
                    {
                        EdgeRight[row3]=line3+1;
                        EdgeRightceshi[row3]=line3+1;
                        Right_F=1;
                        break;	 
                    }
                }

            }
        }
        if(Right_F==0)//�ұ߶���
        {
            EdgeRight[row3]=EdgeRight[row3+1];
            EdgeRightceshi[row3]=RL;   

        }
    	///////////////////Բ�������ж�////////////////////
    	if(yuan_flag==1)
        {
            if(image_dec[8][38]==0||image_dec[9][38]==0||image_dec[10][38]==0||
            image_dec[11][38]==0||image_dec[12][38]==0||image_dec[13 ][38]==0)
            {
                if(Left_F==0&&Right_F==0)//˫�߶���
                {
                    EdgeRight[row3]=50;
                    EdgeRightceshi[row3]=50;
                }
            }

        }

        if(Left_F==1&&Right_F==1)//ֱ���ж��ϰ�
        {
        ///////////////��������Ѳ��/////////////////
            for(row4=59;row4>=30;row4--)
            {
                for(line4=79;line4>0;line4--)
                {
                    if(image_dec[row4][line4-1]==0&&image_dec[row4][line4]==1)
                    {
                        R_barrier_black1=line4;
                        R_find_write_black=1;//Ѱ���׵��ڵĵ�
                        break;
                    }
                }
                for(line4=R_barrier_black1;line4>0;line4--)//����������
                {
                    if(image_dec[row4][line4-1]==1&&image_dec[row4][line4]==0)
                    {
                        R_barrier_black2=line4;       
                        R_find_black_write =1;//�ҵ��ڵ��׵ĵ�
                        break;

                    }
                }
            }
            if(R_find_write_black==1&&R_find_black_write==1)
            {
                if((R_barrier_black1-R_barrier_black2)>16&&(R_barrier_black1-R_barrier_black2)<45)//�������������������
                {
                    if((R_barrier_black2-40)>0)
                    {
                        R_barrier=1;//���ϰ�
                        break;
                    }
                    if((R_barrier_black1-40)<0)
                    {
                        L_barrier=1;//���ϰ�
                        break;
                    }
                }
            }
        }
    ////////���Ҷ���״̬����///////////////
        Left_F=0;
        Left_RF=0;
        Right_F=0;
        Right_LF=0;
    ///////���߼���//////////////
        centerlineceshi[row3]=(EdgeRightceshi[row3]+EdgeLeftceshi[row3])/2;
    //centerlineceshi[row3]=(EdgeRight[row3]+EdgeLeft[row3])/2;

    ////////////��������//////////////
        image_dec[row3][centerlineceshi[row3]]=0;
    }
    if(R_barrier==1)
    {
        //����⵽���ϰ�����ͼ������ƫ��
        for(row=59;row>0;row--)
        {
            centerlineceshi[row]=20;
        }
    }

    if(L_barrier==1)
    {
        //����⵽���ϰ�����ͼ������ƫ��
        for(row=59;row>0;row--)
        {
            centerlineceshi[row]=60;
        }
    }
    Derror();//ƫ�����

}
void Qipaoxian(void)//�������ж�
{
    LED2=LED_ON;
    int row=0,line=0;
	int count=0;
	for(row=59;row>50;row--)
	{
        for(line=79;line>1;line--)
        {
            if(image_dec[row][line]==1&&image_dec[row][line-1]==0)//����������Ѱ�׺������
            {
                count++;//���������м���
            	
            }
        }
        if(count>=7)
        {
            flag=1;
            break;
        }
        else
        {
            count=0;
        }
	}
}
void Derror(void)
{
    uint8_t H,W;
    for(H=59;H>1;H--)
    {
        mid_line_sum += centerlineceshi[H] * quanzhi[H];
        quanzhi_sum += quanzhi[H];
    }
    
    if(quanzhi_sum != 0)
    {
        mid_line_mean = mid_line_sum/quanzhi_sum;
    }
    
    if(mid_line_mean == 0)
    {
        mid_line_mean = 80;
    }

    derror = 39-mid_line_mean;
    fx_sxt=(derror)*fx_p;

    setspeed== g_HighestSpeed -derror * derror* (g_HighestSpeed - g_LowestSpeed)/100;//�ٶ����
}
void SXT(void)
{
    fx_gy=(ENC03_x-GYRO_VAL_x)*fx_d;
    fx_out=fx_gy+fx_sxt;

    if((fx_out>-30&&fx_out<0)&&(fx_out<30&&fx_out>0)) fx_out=0;
    Dir_old=Dir_new;
    Dir_new=fx_out;
}

void DirControlOutput(void)
{
    float fValue;
    fValue=Dir_new-Dir_old;
    Dir_OUT = fValue * (Dir_ControlPeriod + 1) /5 +Dir_old;
}
void SPControlOutput(void)
{
    float fValue;
    fValue=SP_out_new-SP_out_old;
    SP_out=fValue*(SP_ControlPeriod+1)/20 +SP_out_old;
}
void pit_Init(void)
{
    PIT_QuickInit(HW_PIT_CH0, 1000);
    PIT_CallbackInstall(HW_PIT_CH0, PIT0_CallBack); //??????
    PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE); //????0????


}
void Stopcar(void)
{
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH0, 5000 ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH3, 5000 ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH1, 5000 ); 
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH2, 5000 ); 
}

double KalmanFilter(double angleout , double newAngle, double newRate,double dt)
{
    angleout += dt * (newRate - x_bias);
    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt; //??????(dt)^2?????
    P_01 +=  - dt * P_11;
    P_10 +=  - dt * P_11;
    P_11 +=  + Q_gyro * dt;
    y = newAngle - angleout;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    angleout +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00; 
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    return angleout;
}