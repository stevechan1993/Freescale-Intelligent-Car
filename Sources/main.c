#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <stdio.h>
/************************************************************************************ 

                          一·    全局变量声明模块

*************************************************************************************/  
typedef unsigned char INT8U;
typedef unsigned int INT16U;
typedef  int INT32; 
typedef struct 
{ 
  INT8U d; //存放这一次AD转换的值              
}DATA;
/****************************************************
             全局变量声明区
             
*****************************************************/
DATA data[8]={0};                 //全局变量数组，存放赛道AD转换最终结果
INT8U a[8][8]={0};                //全局变量用来存放赛道AD转换中间结果
INT8U cross0,cross1;              //记录十字叉线
INT8U value;                      //读编码器周期
#define duojmax 1690              //向左转向最大值
#define duojmid 1440              //打在中间
#define duojmin 1210              //向右转向最大值
#define dianjmax 250
#define dianjmin 20
#define dianjmid 125
#define dianjmax0 250
#define dianjmin0 20
#define dianjmid0 125

int road_change[100]={0};         //判断赛道情况数组
int road_change1[100]={0};         //判断赛道情况数组
int *r_change0;                   //指向数组最后一位
int *r_change1;                   //指向数组倒数第二位
int *r_change2;                   //指向数组最后一位
int *r_change3;                   //指向数组倒数第二位
int sum_front=0,sum_back=0;       //分别存储数组前后两部分的和
int sum_front1=0,sum_back1=0;       //分别存储数组前后两部分的和
int m,n;
int sum1=0;
int sum2=0;
int change,change1,change2;
/******************************速度测量参数定时********************************/
#define PIT0TIME 800              //定时0初值：  设定为 4MS 测一次速度，采一次AD值   //800
#define PIT1TIME 1390             //定时1初值： 设定为 7ms定时基值   //1390
/*******************************脉冲记数变量*******************************/
static INT16U PulseCnt,PulseCnt0;          //最终的脉冲数
/******************************电机PID变量*********************************/
struct 
{
   int error0;
   int error1;
   int error2;
   int speed;
   int chage;
   float q0,q1,q2,Kp,Kd,Ki;
}static SpeedPid,SpeedPid0;
/********************************速度变量设定*******************************/
static int NowSpeed1=0;
static int NowSpeed2=0;
static int NowSpeed3=0;
INT8U breaktime=0;           //刹车时间
INT8U backflag=0;             //刹车开关 0:关闭刹车 1:开启刹车
static INT16U break_pwm0=0;  //刹车力度
static INT16U break_pwm=0;  //刹车力度
static int speed_control,speed_control0;  //存储pid输出值
static int speed_return,speed_return0,speed1,speed2,rate1,rate2;
/*******************************舵机PID参数******************************/
struct
{
   int error0;
   int error1;
   int error2;
   int chage;
   float q0,q1,q2,Kp,Kd,Ki;
}PositionPid;
static INT16U angle_control=duojmid;      //舵机PWM最终控制量
static INT16U angle_control0=duojmid;
static INT16U angle_control1=duojmid;
/****************************oled液晶显端口定义**************************/
#define LCD_SCL   PORTB_PB0  
#define LCD_SDA	  PORTB_PB1
#define LCD_RST   PORTB_PB2
#define LCD_DC    PORTB_PB3  
#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xCF
#define X_WIDTH 128
#define Y_WIDTH 64
INT16U Counter=0;
INT8U select=0;
/****************************CCD端口定义**************************/
#define TSL_SI  PTS_PTS1    //定义线性传感器的端口 SI
#define TSL_CLK PTS_PTS0   //定义线性传感器的端口 CLK
byte ADV[128]={0,0};         //声明数组，用于存放采集的线性数值
word LCDD[128]={0,0};        //转换为LCD显示的数值
/**************************标志变量区*************************************/
INT8U stop_flag=0;
INT8U start_flag=0;
INT8U flag=0;                //读编码器
INT8U zhijwan=0;             //直道进弯道标志位
INT8U tap=0;
INT8U Obstruct_flag_l=0;
INT8U choise=0;
INT8U Obstruct_flag_r=0;
INT16U Cnt=0;
/************************************************************************************ 

                            二·    初始化函数模块

*************************************************************************************/  

/**************************************************************

           1.   芯片初始化--------MCUInit（）
                      
**************************************************************/
void MCUInit(void)
{
 //////////////////////////////////////////////////////////////////////////////////////////
 //                ********总线周期计算方法 ********                                     //
 //                fBUS=fPLL/2                                                           //
 //                fvoc=2*foscclk*(synr+1)/(refdv+1)                                     //
 //                PLL=2*16M*(219+1)/(69+1)=96Mhz                                        //
 //                                                                                      //   
///////////////////////////////////////////////////////////////////////////////////////////
  CLKSEL=0X00;
  PLLCTL_PLLON=1;              //锁相环控制
  SYNR = 0X40|0X05;
  REFDV =0X80|0X01;
  POSTDIV=0X00;
  while( CRGFLG_LOCK != 1);    //等待锁相环时钟稳定，稳定后系统总线频率为24MHz
  CLKSEL_PLLSEL = 0x01;        //选定锁相环时钟
  PLLCTL=0xf1;                 //锁相环控制
                               //时钟合成 fpllclk=2*foscclk*(synr+1)/(refdv+1)
                               //synr=2;refdv=1;外部时钟foscclk=16mb
                               //fpllclk=48mb 总线时钟24mb
}
 /**************************************************************

           2.   AD转换初始化--------ADCInit（）
                      
**************************************************************/
void ADCInit(void)
{ 
  ATD0CTL1=0x00; 
  ATD0CTL2=0x40;    //0100,0000，自动清除使能控制位，忽略外部触发
                    //转换结束允许中断，中断禁止
  ATD0CTL3=0xA4;    //0100,0100，转换序列长度为4； FIFO模式,冻结模式下继续转换
  ATD0CTL4=0x05;    //00001000,8位精度,PRS=5，ATDCLOCK=BusClock(24mb)/(5+1)*2,约为2MHz,采样周期位4倍AD周期
  ATD0DIEN=0x00;    //输入使能禁止 
}
/**************************************************************

           3.   PWM初始化--------PWMInit（）
                      
**************************************************************/
void PWMInit(void) //PWM初始化
{ 
  //总线频率24mb
  //1. 选择时钟：PWMPRCLK,PWMSCLA,PWMSCLB,PWMCLK 
  PWME=0x00;                     //PWM通道关闭
  PWMPRCLK=0x00;			           //时钟源A=BusClockA/1=48MB/1=48MB;时钟源B=BusClockA/1=48MB/1=48MB
  //低位clockA:0,1,4,5;高位clockB:2,3,6,7    
  PWMSCLA=6;                     //ClockSA=ClockA/2/6=48MB/12=4MB
  PWMSCLB=24;                    //ClockSB=ClockB/2/24=48MB/48=1MBHz
  PWMCLK =0xFF;			               //通道均级联，均用SA,SB ,且都为6MB 
  //2. 选择极性： PWMPOL
  PWMPOL = 0xff;			             //电机正反转寄存器（PWMPOL）起始输出为高电平 
  //3. 选择对齐方式：PWMCAE
  PWMCAE=0x00;  //输出左对齐
  //4.PWMCTL PWM控制寄存器
  //PWMCTL=0xF0;			             //01,23,45,67通道都级连 ，输出风别由1，3，5，7口控制
  PWMCTL_CON67=1;                  //23通道级连，输出由3口控制    
  //5. 对占空比和周期编程 
  //周期计算公式 ： 输出周期=通道时钟周期*（PWMPERX+1）
  //占空比：=(PWMPERYX+1)/（PWMPERX+1）
  //开始时刻应使舵机打直，电机不转
  //1.通道45用来控制舵机PWM
  PWMPER67=20000-1;			         //PWM23=1MB/(20000)=50Hz
  //1.通道0,1用来控制左电机
  //2.通道4,5用来控制右电机
  PWMPER0=250-1;                   //电机正转PWM周期初始化。pwm0=4MB/(250)=16khz  
  PWMPER1=250-1;                   //电机反转PWM周期初始化。pwm1=4MB/(250)=16khz
  PWMPER5=250-1;                   //电机正转PWM周期初始化。pwm5=4MB/(250)=16khz  
  PWMPER4=250-1;                   //电机反转PWM周期初始化。pwm4=4MB/(250)=16khz
  //6. 使能PWM通道; PWME
  PWME=0XFF;                       //通道使能开
}
/**************************************************************

           4.   测速模块初始化--------ECTInit（）
                      
**************************************************************/
void Pulse_int(void)    //脉冲累加器初始化
{
  PACTL=0X50;      //PAI使能，PT7计数
  TCTL3=0x80;      //PT7脚捕捉上升沿        
  TSCR2_PR=3;		  //时间周期为8分频
  TFLG1=0xFF;		  //清除该位
  TFLG2=0x80;		  //清除 TOF      
  TIOS_IOS7=0;     //PT7口为输入捕捉
  TSCR1_TEN = 1;   //定时器使能
  PACNT=0x00;      //clear
}
void PIT_Init(void)// 定时中断初始化函数 50MS 定时中断设置
{
///////////////////////////////////////////////////////////////////////// 
  PITCFLMT_PITE=0;    // PIT定时模块使能关
  PITCFLMT_PFLMT0=1;  //锁定8位微定时器0
  PITMTLD0=240-1;     //8 位定时器初值设定。 240 分频，在 48MHzBusClock 下，为即5us.

//-----------PIT0初始化

  PITCE_PCE0=0;       // 定时器通道 0 使能
  PITMUX_PMUX0=0;     //channel 0 connected to micro timer 0
  PITLD0=PIT0TIME-1;  //16 位定时器初值设定。 PITTIME*5US=800*5us=4ms
  PITCE_PCE0=1;       //通道0使能  
  PITINTE_PINTE0=1;   //开通PIT0定时器的溢出中断    

//-----------PIT1初始化----产生1MS基定时
  PITCE_PCE1=0;   
  PITMUX_PMUX1=0;
  PITLD1=PIT1TIME-1;  // PITTIME*5US=*5us=7ms
  PITCE_PCE1=1;       //通道1使能 
  PITINTE_PINTE1=1;   //开通PIT0定时器的溢出中断

  PITCFLMT_PITE=1;    // PIT定时模块使能开
  TSCR1_TEN = 1;      //定时器使能
}
/*************************************************************
           6.输入输出口初始化-------IOInit()

**************************************************************/
void IOInit(void) 
{
  DDRA=0X0C; //A口定义为输出  0XFF
  DDRB=0X0F; //B口定义为输出  0XFF
  DDRK=0X00; //k口定义为输入
  DDRM=0XFF;
  DDRS=0XFF;
  PORTB=0X00;
  PORTA=0X00;
}
/************************************************************
                    中断优先级设置

*************************************************************/
void interrupt_first(void) 
{
  INT_CFADDR=0x70;              //中断优先级设置
  INT_CFDATA4_PRIOLVL=1;       //pit1
  INT_CFDATA5_PRIOLVL=5;       //PIT0
}
//-----------------------------------------------------
//延时 based on an 48MHz clock
//k=1 时约为1ms定时  
void delay(INT8U k)
{       
  INT8U i;
  INT16U j;
  for(i=0;i<40;i++)
    for(j=0;j<100*k;j++)
      asm("nop");
}
//delay for 100us
void delay_100us(void)
{
  INT8U i,j;
  for(i=0;i<6;i++)
    for(j=0;j<100;j++)
      asm("nop");
}
//delay for 5ms
void delay_5ms(void)
{
  INT8U I;
  INT16U J;
  for(I=0;I<6;I++)
    for(J=0;J<5000;J++)
      asm("nop");
}
//开始起跑延时
void start_delay(void) 
{  
  Counter=0;
  while(Counter<550)asm(nop);
  start_flag=1;    //开始起跑
  Counter=0;       //计数器置零
  select=0;        //秒计时器置零
}
//1ms时基刹车
void brake_1ms(unsigned int xms)
{
  int i,j;
  PWMDTY1=0;
  PWMDTY4=0;
  asm(nop);
  asm(nop);
  asm(nop);
  for(i=0;i<40;i++) 
  {
    for(j=0;j<100*xms;j++) 
    {
      PWMDTY0=200;
      PWMDTY5=200;
    }
  }
  PWMDTY0=0;
  PWMDTY5=0;
}

/*************************************************************************
                   oled液晶显示模块程序
 DM-162采用标准的14脚接口，其中VSS为地电源，VDD接5V正电源，V0为液晶显示器
 对比度调整端，接正电源时对比度最弱，接地电源时对比度最高，对比度过高时会
 产生"鬼影"，使用时可以通过一个10K的电位器调整对比度。RS为寄存器选择，高电
 平时选择数据寄存器、低电平时选择指令寄存器。RW为读写信号线，高电平时进行
 读操作，低电平时进行写操作。当RS和RW共同为低电平时可以写入指令或者显示地
 址，当RS为低电平RW为高电平时可以读忙信号，当RS为高电平RW为低电平时可以写
 入数据。E端为使能端，当E端由高电平跳变成低电平时，液晶模块执行命令;
 D0~D7为8位双向数据线。
**************************************************************************/
unsigned char F6x8[][6]=
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp  0
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !   1
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "   2
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #   3
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $   4
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %   5
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &   6
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '   7
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (   8
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )   9
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *   10
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +   11
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,   12  
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -   13
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .   14
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /   15
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0   16
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1   17
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2   18
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3   19
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4   20
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5   21
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6   22
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7   23
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8   24
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9   25
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :   26
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;   27
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <   28
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =   29
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >   30
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?   31
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @   32
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A   33
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B   34
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C   35
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D   36
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E   37
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F   38
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G   39
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H   40
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I   41
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J   42
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K   43
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L   44                               
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M   45
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N   46
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O   47
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P   48
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q   49
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R   50
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S   51
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T   52
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U   53
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V   54
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W   55
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X   56
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y   57
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z   58
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [   59
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55  60
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]   61
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^   62
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _   63
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '   64
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a   65
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b   66
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c   67
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d   68
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e   69
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f   70
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g   71
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h   72
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i   73
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j   74
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k   75
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l   76
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m   77
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n   78
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o   79
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p   80
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q   81
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r   82
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s   83
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t   84
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u   85
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v   86
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w   87
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x   88
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y   89
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z   90
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines  91
};

//======================================================
// 128X64I液晶底层驱动[8X16]字体库
// 设计者: powerint
// 描  述: [8X16]西文字符的字模数据 (纵向取模,字节倒序)
// !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//======================================================

unsigned char F8X16[]=
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x0,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94

};    
/*===========================================================
                        屏幕延时程序
=============================================================*/
void LCD_DLY_ms(int ms)
{                         
	unsigned int a;
	while(ms)
	{
		a=1335;
		while(a--);
		ms--;
	}
	return;
}

/*===========================================================
                        屏幕数据输入
=============================================================*/
void LCD_WrDat(unsigned char dat)
{
	unsigned char i=8;
	LCD_DC=1;
	LCD_SCL=0; 
	while(i--)
	{
	    if(dat&0x80){LCD_SDA=1;}
	    else{LCD_SDA=0;}
	    LCD_SCL=1;            
	    LCD_SCL=0;    
	    dat<<=1;    
	}
} 
/*===========================================================
                       屏幕地址设置初始化
=============================================================*/
void LCD_WrCmd(unsigned char cmd)
{
	  unsigned char i=8;
  	LCD_DC=0;
  	LCD_SCL=0;   
  	while(i--)
  	{
	    if(cmd&0x80){LCD_SDA=1;}
	    else{LCD_SDA=0;}
	    LCD_SCL=1;            
	    LCD_SCL=0;    
	    cmd<<=1;   
  	} 	
}
/*===========================================================
                          屏幕清空
=============================================================*/
void LCD_Fill(unsigned char bmp_dat)
{
	unsigned char y,x;
	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(bmp_dat);
	}
}
/*===========================================================
                    输入起始位置         
=============================================================*/
void LCD_Set_Pos(char x,char y)
{ 
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd((x&0x0f)|0x01); 
}
/*===========================================================
                          
=============================================================*/
void LCD_CLS(void)
{
	unsigned char y,x;	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10); 
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(0);
	}
} 
/*===========================================================
                          屏幕初始化
=============================================================*/

void LCD_Init(void)        
{  
	LCD_SCL=1;
	//LCD_CS=1;	//预制SLK和SS为高电平  	
	LCD_RST=0;
	LCD_DLY_ms(50);
	LCD_RST=1;
	
	//从上电到下面开始初始化要有足够的时间，即等待RC复位完毕   
  
  LCD_WrCmd(0xae);//--turn off oled panel 关掉oled面板
  LCD_WrCmd(0x00);//---set low column address 设置低列地址
  LCD_WrCmd(0x10);//---set high column address 设置高列地址
  LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F) 设置开始行地址集映射RAM显示开始线(0 x00 ~ 0 x3f)
  LCD_WrCmd(0x81);//--set contrast control register 设置对比控制寄存器
  LCD_WrCmd(0xcf); // Set SEG Output Current Brightness 集凹陷输出电流的亮度
  LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常   集凹陷/列映射
  LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常   设置COM /行扫描方向
  LCD_WrCmd(0xa6);//--set normal display    设置正常显示
  LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64) 设置多路比(1到64)
  LCD_WrCmd(0x3f);//--1/64 duty 1/64的责任
  LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F) 设置显示抵消转变映射RAM计数器(0 x00 ~ 0 x3f
  LCD_WrCmd(0x00);//-not offset 不是抵消
  LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency  设置显示时钟分比/振荡器频率
  LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec 设置分比、设置时钟作为100帧/秒
  LCD_WrCmd(0xd9);//--set pre-charge period 设置预充电周期
  LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock 设置前收取15钟&放电作为1时钟
  LCD_WrCmd(0xda);//--set com pins hardware configuration 设置com别针硬件配置
  LCD_WrCmd(0x12);
  LCD_WrCmd(0xdb);//--set vcomh 设置vcomh
  LCD_WrCmd(0x40);//Set VCOM Deselect Level 集威科姆公司取消选定水平
  LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02) 设置页面寻址模式(0 x00/0x01/0x02)
  LCD_WrCmd(0x02);// 
  LCD_WrCmd(0x8d);//--set Charge Pump enable/disable 设置电荷泵启用/禁用
  LCD_WrCmd(0x14);//--set(0x10) disable 集(0 x10)禁用
  LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5) 禁用整个显示在(0 xa4/0xa5)
  LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)  禁用反显示在(0 xa6 / a7)
  LCD_WrCmd(0xaf);//--turn on oled panel 打开oled面板
  LCD_Fill(0x00);  //初始清屏
  LCD_Set_Pos(0,0); 
  LCD_Fill(0x00);	
}
//==============================================================
//函数名：LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P6x8Str(char x,char y,char ch[])
{
  unsigned char c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	  LCD_WrDat(F6x8[c][i]);  
  	x+=6;
  	j++;
  }
}
//==============================================================
//函数名：LCD_P6x8num(unsigned char x,unsigned char y,unsigned int *p)
//功能描述：写入一整形变量
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P6x8num(char x,char y,int num)
{
  unsigned char i=0,j=0;
  int n=0,num1=0,c=0,js=1,js1;
  if(num<0)
  {
    num=-num;
    num1=num;
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	LCD_WrDat(F6x8[13][i]);
  	x=x+8; 
  }
  else
  {
    num1=num;
  }
  
  if(num==0) 
  {
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	LCD_WrDat(F6x8[16][i]);
  }
  while(num1!=0) 
  {
    num1 = num1/10;
    n++;
  }
  for(js1=n-1;js1>0;js1--) 
  {
    js=js*10;
  }
  while (n!=0)
  { 
    c =num/js;
  	num=num-c*js;
  	c=c+16;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	LCD_WrDat(F6x8[c][i]); 
  	x+=6;
  	js=js/10;	
  	n--;
  }
  for(j=1;j>0;j--)
  {
    for(i=0;i<8;i++)     
  	LCD_WrDat(F6x8[0][i]);                                                         
  }
}
//==============================================================
//函数名：LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//============================================================== 
 
void LCD_P8x16Str(char x,char y,char ch[])
{
  unsigned char c=0,i=0,j=0;
        
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<8;i++)     
  	  LCD_WrDat(F8X16[c*16+i]);
  	LCD_Set_Pos(x,y+1);    
  	for(i=0;i<8;i++)     
  	  LCD_WrDat(F8X16[c*16+i+8]);  
  	x+=8;
  	j++;     
  }
}

void LED_disp(void) 
{  
  LCD_P6x8num(50,2,data[5].d);   //C
        
  LCD_P6x8num(15,3,data[4].d);   //L
  LCD_P6x8num(50,3,change);
  LCD_P6x8num(85,3,data[6].d);   //R
  LCD_P6x8num(15,4,(sum_front+sum_back));
  LCD_P6x8num(85,4,(sum_front-sum_back));
  
  LCD_P6x8num(15,5,data[3].d);   //LT
  LCD_P6x8num(50,5,change1);
  LCD_P6x8num(85,5,data[7].d);   //RT
  LCD_P6x8num(15,6,(sum_front1+sum_back1));
  LCD_P6x8num(50,6,tap);
  LCD_P6x8num(85,6,(sum_front1-sum_back1));
  
  LCD_P6x8num(15,7,sum1);   
  LCD_P6x8num(50,7,(sum1-sum2));
  LCD_P6x8num(85,7,sum2);
}
/************************************************************************************ 

                             三 ·    子函数模块

*************************************************************************************/  
/************************************************************************
                    
                      CCD转换包括函数;
                     
/ ***********************************************************************/
/*************************不是很精确，保证一定的数量级**************************/
void Dly_us(byte us)
{
   byte ii;    
   for(ii=0;ii<us;ii++)
   {
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop); 
      
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);
      _asm(nop);_asm(nop);_asm(nop);_asm(nop);  
   }          
}
/***********************读CCD***********************/
void RD_TSL(void) 
{
  byte i=0,tslp=0;
  
  TSL_CLK=1;//起始电平高 
  TSL_SI=0; //起始电平低
  delay(1); //合理的延时
      
  TSL_SI=1; //上升沿
  TSL_CLK=0;//下降沿
  delay(1); //合理延时
      
  TSL_CLK=1;//上升沿
  TSL_SI=0; //下降沿
  delay(1); //合理延时 
       
  for(i=0;i<64;i++)
  { 
    TSL_CLK=0;//下降沿    
    Dly_us(8-i/8+1); //合理延时
    while(!ATD0STAT0_SCF);//等待转换结束
    ADV[tslp]=ATD0DR0L;  //AD采集
    ++tslp;
    TSL_CLK=1;//上升沿 
    Dly_us(8-i/8+1); //合理延时    
  }
  for(i=0;i<64;i++)
  { 
    TSL_CLK=0;//下降沿    
    Dly_us(i/8+1); //合理延时
    while(!ATD0STAT0_SCF);//等待转换结束
    ADV[tslp]=ATD0DR0L;  //AD采集
    ++tslp;
    TSL_CLK=1;//上升沿 
    Dly_us(i/8+1); //合理延时    
  }
  for(m=0;m<64;m=m+1) 
  {
     sum1+=ADV[m];
  }
  sum1=sum1/64;
  for(n=65;n<128;n=n+1) 
  {
     sum2+=ADV[n];
  }
  sum2=sum2/64;
  change2=sum1-sum2;
}
/***************************单行显示***************************/
void SHOW_TSL8(void) 
{    
  byte i=0,maxv=0,minv=0xff;
  word evv=0,tslp=0;
  char txt[16]="";   
  for(i=0;i<128;i++)
  { 
    evv+=ADV[i];           
  }
  evv=evv/128;
  for(i=0;i<128;i++)                         
  {
    if(minv>ADV[i])  minv=ADV[i];
    if(maxv<ADV[i])  maxv=ADV[i];               
  }
  (void)sprintf(txt,"Max:%03d",maxv);
  LCD_P6x8Str(0,1,txt);                        
  
  (void)sprintf(txt,"Evv:%03d",evv);
  LCD_P6x8Str(43,1,txt);   
  
  (void)sprintf(txt,"Min:%03d",minv);
  LCD_P6x8Str(85,1,txt);                    
  LCD_Set_Pos(0,0);    
  for(i=0;i<128;i++)                        
  {
    if(ADV[i]<= evv-evv/5)  tslp=0xfe;        
    else tslp=0x80;      
    LCD_WrDat(tslp);           
  }
}   
/************************************************************************
                    
                      AD转换包括函数MAX(),MIN(),fun_ADC();
                     
/ ***********************************************************************/

INT8U MAX(INT8U k)  
{
  INT8U i,*q,max;
  q=a[k];
  max=*q;
  for(i=0;i<8;i++) 
    if(*(q+i)>max)
      max=*(q+i);  
  return max;
}

INT8U MIN(INT8U m)
{
  INT8U j,min,*p;
  p=a[m];
  min=*p;
  for(j=0;j<8;j++)
    if(*(p+j)<min)
      min=*(p+j);
  return min;
}
                                   
 /******************************************************************** 
                    经过计算0.50ms即450us可完成一次AD转换
                    AD3ms定时刷新一次
 *********************************************************************/ 
       
void  fun_ADC(void)
{
  INT8U i,j,max,min;                       
  INT16U sum;
  INT8U *q;
  ATD0CTL5=0X20;    //,连续AD转换，右对齐，单通道，无符号，起始通道为AN0
  while(!ATD0STAT0_SCF);//等待左前第一个电感电压转换完成
  q=a[0];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  ATD0CTL5=0X21;                                                     
  while(!ATD0STAT0_SCF); //右前侧第一个电感电压转换完成  
  q=a[1];  
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;

  ATD0CTL5=0X22;
  while(!ATD0STAT0_SCF);  //左前侧第二个电感电压转换完成 
  q=a[2]; 
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  ATD0CTL5=0X23;
  while(!ATD0STAT0_SCF);//  右前侧第1个电感电压转换完成
  q=a[3];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  ATD0CTL5=0X24;
  while(!ATD0STAT0_SCF);//等待左前第一个电感电压转换完成
  q=a[4];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;

  ATD0CTL5=0X25;
  while(!ATD0STAT0_SCF);//  右前侧第1个电感电压转换完成
  q=a[5];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  
  ATD0CTL5=0X26;
  while(!ATD0STAT0_SCF);
  q=a[6];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  
  ATD0CTL5=0X27;
  while(!ATD0STAT0_SCF);
  q=a[7];
  *(q)  =ATD0DR0L;
  *(q+1)=ATD0DR1L;
  *(q+2)=ATD0DR2L;
  *(q+3)=ATD0DR3L;
  /*********************************************************
          再次启动一轮AD转换来减小误差  

  ***********************************************************/
  ATD0CTL5=0X27;
  while(!ATD0STAT0_SCF);
  q=a[7];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;
  
  ATD0CTL5=0X26;
  while(!ATD0STAT0_SCF);
  q=a[6];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;
  
  ATD0CTL5=0X25;
  while(!ATD0STAT0_SCF);//  右前侧第1个电感电压转换完成
  q=a[5];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;
  ATD0CTL5=0X24;
  while(!ATD0STAT0_SCF);//等待左前第一个电感电压转换完成
  q=a[4];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;
  ATD0CTL5=0X23;
  while(!ATD0STAT0_SCF);//  右前侧第1个电感电压转换完成
  q=a[3];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;

  ATD0CTL5=0X22;
  while(!ATD0STAT0_SCF);  //左前侧第二个电感电压转换完成 
  q=a[2]; 
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;

  ATD0CTL5=0X21;    //重置ATD0CTL5,将启动一次新的AD转换
  while(!ATD0STAT0_SCF); //右前侧第一个电感电压转换完成  
  q=a[1];  
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;

  ATD0CTL5=0X20;    //,连续AD转换，右对齐，单通道，无符号，起始通道为AN0
  while(!ATD0STAT0_SCF);//等待左前第一个电感电压转换完成
  q=a[0];
  *(q+4)=ATD0DR0L;
  *(q+5)=ATD0DR1L;
  *(q+6)=ATD0DR2L;
  *(q+7)=ATD0DR3L;

  for(i=0;i<8;i=i+1)                       //累加后求均值，AD转换滤波
  {
    sum=0;
    q=a[i];     //赋予指针新地址
    for(j=0;j<8;j++)
      sum+=*(q+j);
    max=MAX(i);
    min=MIN(i);
    data[i].d=(INT8U)((sum-max-min)/6);
  }
  //1.记录100个0，5号控制电感的偏差值

  r_change0=&road_change[99];   //指向数组最后一位
  r_change1=&road_change[98];   //指向数组倒数第二位
  while(r_change1!=road_change) 
  {
    *r_change0=*r_change1;
    r_change0--;
    r_change1--;//指针前移
  }
  *r_change0=*r_change1;
  *r_change1=data[4].d-data[6].d;

  //1.做和识别赛道
  sum_front=0;
  sum_back=0; 
  r_change0=road_change;   //指向数组第一个数
  r_change1=&road_change[99];   //指向数组倒数第一个数
  while(r_change0<r_change1) 
  {
    sum_front+=(*r_change0>=0?*r_change0:-*r_change0);  //绝对值求和
    sum_back +=(*r_change1>=0?*r_change1:-*r_change1);
    r_change0++;//指针后移
    r_change1--;//指针前移
  }
  //1.记录100个3，2号控制电感的偏差值
  r_change2=&road_change1[99];   //指向数组最后一位
  r_change3=&road_change1[98];   //指向数组倒数第二位
  while(r_change3!=road_change1) 
  {
    *r_change2=*r_change3;
    r_change2--;
    r_change3--;//指针前移
  }
  *r_change2=*r_change3;
  *r_change3=data[3].d-data[7].d;
  //2.做和识别赛道
  sum_front1=0;
  sum_back1=0; 
  r_change2=road_change1;   //指向数组第一个数
  r_change3=&road_change1[99];   //指向数组倒数第一个数
  while(r_change2<r_change3) 
  {
    sum_front1+=(*r_change2>=0?*r_change2:-*r_change2);  //绝对值求和
    sum_back1 +=(*r_change3>=0?*r_change3:-*r_change3);
    r_change2++;//指针后移
    r_change3--;//指针前移
  }
  change=data[4].d-data[6].d;
  change1=data[3].d-data[7].d;
}
void speedchoise(void) 
{
  choise=PORTB&0xf0;
  switch(choise) 
  {
  //******1档：匀速前进速度约为m:/s******
    case 208://key2
    {
      tap=2;
      NowSpeed1=160;
      NowSpeed2=150;
      NowSpeed3=150;
      breaktime=15;        //刹车时间
      backflag=1;          //刹车开关 0:关闭刹车 1:开启刹车
      break_pwm=180;       //刹车力度
      break_pwm0=180;       //刹车力度
      rate1=1300;
      rate2=430;
      speed1=50;
      speed2=50;
    }break;
    //******2档: 直道速度约为：m/s，弯道保持的变速方案1******
    case 176://key3
    {
      tap=3;
      NowSpeed1=180;
      NowSpeed2=150;
      NowSpeed3=150;
      breaktime=15;        //刹车时间
      backflag=1;          //刹车开关 0:关闭刹车 1:开启刹车
      break_pwm=200;       //刹车力度
      break_pwm0=200;       //刹车力度
      rate1=1300;
      rate2=430;
      speed1=50;
      speed2=50;
    }break;
    //******3档: 直道速度约为：m/s，弯道保持的变速方案2******
    case 112://key4
    {
      tap=4;
      NowSpeed1=200;
      NowSpeed2=155;
      NowSpeed3=155;
      breaktime=20;        //刹车时间
      backflag=1;          //刹车开关 0:关闭刹车 1:开启刹车
      break_pwm=200;       //刹车力度
      break_pwm0=200;       //刹车力度
      rate1=1300;
      rate2=430;
      speed1=50;
      speed2=50;
    }break;
    //******其它情况 直道速度约为：m/s，弯道保持的变速方案******
    default:
    { 
      tap=1;
      NowSpeed1=140;
      NowSpeed2=135;
      NowSpeed3=130;
      breaktime=10;        //刹车时间
      backflag=1;          //刹车开关 0:关闭刹车 1:开启刹车
      break_pwm=150;       //刹车力度
      break_pwm0=150;       //刹车力度
      rate1=1200;
      rate2=430;
      speed1=60;
      speed2=60;
    }break;
  }
 return;
}

/****************************************************************
                          舵机pid

*****************************************************************/
void direction_pid_init(void)
{
 /********************舵机PID参数初始化***********************/
  PositionPid.Kp=4;     
  PositionPid.Ki=0.7;    
 
  PositionPid.error0=0;
  PositionPid.error1=0;
  PositionPid.error2=0;
  
  PositionPid.chage=0;
  angle_control=duojmid;
  
  PositionPid.q0=PositionPid.Kp+PositionPid.Ki+PositionPid.Kd; //q0=kp+ki+kd;
  PositionPid.q1=-PositionPid.Kp-2*PositionPid.Kd;          //q1=-(kp+2kd);          
  PositionPid.q2=PositionPid.Kd;                         //q2=Kd;
  
}
/****************************************************************
                          电机pid

*****************************************************************/
//------------------------------------------------------------------
//速度PID系数初始化
//------------------------------------------------------------------
void SpeedPID_Init(void)
{ 
 /********************电机PID参数初始化***********************
      PID参数调节规律kp=kp,  ki=0.0425*kp, kd=0.37125*kp ;
 
 ************************************************************/
 //左电机
  SpeedPid.Kp=30;      
  SpeedPid.Ki=2.5;     
  SpeedPid.Kd=5;
  
  SpeedPid.error0=0;
  SpeedPid.error1=0;     
  SpeedPid.error2=0;
  SpeedPid.chage=0;
 
 
  SpeedPid.q0=SpeedPid.Kp+SpeedPid.Ki+SpeedPid.Kd; //q0=kp+ki+kd;
  SpeedPid.q1=-SpeedPid.Kp-2*SpeedPid.Kd;          //q1=-(kp+2kd);          
  SpeedPid.q2=SpeedPid.Kd;                         //q2=Kd;
  
  SpeedPid0.Kp=30;      
  SpeedPid0.Ki=2.5;     
  SpeedPid0.Kd=5;
  
  SpeedPid0.error0=0;
  SpeedPid0.error1=0;     
  SpeedPid0.error2=0;
  SpeedPid0.chage=0;
  
  SpeedPid0.q0=SpeedPid0.Kp+SpeedPid0.Ki+SpeedPid0.Kd; //q0=kp+ki+kd;
  SpeedPid0.q1=-SpeedPid0.Kp-2*SpeedPid0.Kd;           //q1=-(kp+2kd);          
  SpeedPid0.q2=SpeedPid0.Kd;                           //q2=Kd;
}

void direction_control(void)
{
  if(data[5].d>115)
  { 
    //检测障碍物
    if(change2>15) 
    {
      Obstruct_flag_r=1;     //障碍在右边
      Cnt=0;
    }
    if(change2<-15) 
    { 
      Obstruct_flag_l=1;     //障碍在左边
      Cnt=0;
    }
    if(Obstruct_flag_r==1) 
    {
      change=(int)(2.0*(data[5].d-data[6].d));
      if(Cnt>150) 
      {
        Obstruct_flag_r=0;
        Cnt=0;
      } 
      else
        goto exit;
    }
    if(Obstruct_flag_l==1) 
    {
      change=(int)(2.0*(data[4].d-data[5].d));
      if(Cnt>150) 
      {
        Obstruct_flag_l=0;
        Cnt=0;
      } 
      else
        goto exit;
    } 
    exit:
    PositionPid.error0=change;
    PositionPid.chage=(int)(PositionPid.q0*PositionPid.error0+PositionPid.q1*PositionPid.error1+PositionPid.q2*PositionPid.error2);
    PositionPid.error2=PositionPid.error1;
    PositionPid.error1=PositionPid.error0;
    angle_control=duojmid+PositionPid.chage;
    if(angle_control>duojmax)angle_control=duojmax;
    else if(angle_control<duojmin)angle_control=duojmin;
    PWMDTY67=angle_control-1; 
    if(stop_flag==0&&start_flag!=0)
    {
      speed_return=PulseCnt;
      speed_return0=PulseCnt0;
      if(((sum_front-sum_back)>rate1||(sum_front1-sum_back1)>rate2)&&speed_return>speed1&&speed_return0>speed1)  
      {
        if(backflag==1) 
        {
          PWMDTY4=0;
          PWMDTY1=0;
          asm(nop);
          asm(nop);
          asm(nop);
          PWMDTY5=break_pwm-1;
          PWMDTY0=break_pwm0-1;
        }
        delay(breaktime);
      }else;
      PWMDTY5=0;  
      PWMDTY0=0;  
      
      SpeedPid.error0=NowSpeed1-speed_return;
      SpeedPid0.error0=NowSpeed1-speed_return0;
      
      SpeedPid.chage=(int)(SpeedPid.q0*SpeedPid.error0+SpeedPid.q1*SpeedPid.error1+SpeedPid.q2*SpeedPid.error2);
      SpeedPid.error2=SpeedPid.error1;
      SpeedPid.error1=SpeedPid.error0;
      SpeedPid.speed=NowSpeed1+SpeedPid.chage;
      
      SpeedPid0.chage=(int)(SpeedPid0.q0*SpeedPid0.error0+SpeedPid0.q1*SpeedPid0.error1+SpeedPid0.q2*SpeedPid0.error2);
      SpeedPid0.error2=SpeedPid0.error1;
      SpeedPid0.error1=SpeedPid0.error0;
      SpeedPid0.speed=NowSpeed1+SpeedPid0.chage; 
      
      if(SpeedPid0.speed>NowSpeed1)SpeedPid0.speed=NowSpeed1;
      if(SpeedPid.speed>NowSpeed1)SpeedPid.speed=NowSpeed1;
      
      speed_control=SpeedPid.speed;
      speed_control0=SpeedPid0.speed;
      
      PWMDTY1=speed_control0-1;    
      PWMDTY4=speed_control-1;    
    }
  } 
  else if(data[5].d<115&&data[5].d>=5) 
  {
    if(change>0) 
    {
      change=385-change;
      PositionPid.error0=change;
      PositionPid.chage=(int)(PositionPid.q0*PositionPid.error0+PositionPid.q1*PositionPid.error1+PositionPid.q2*PositionPid.error2);
      PositionPid.error2=PositionPid.error1;
      PositionPid.error1=PositionPid.error0;
      angle_control=duojmid+PositionPid.chage;
      if(angle_control>duojmax)angle_control=duojmax;
      else if(angle_control<duojmin)angle_control=duojmin;
      PWMDTY67=angle_control-1; 
    }
    if(change<0) 
    {
      change=-385-change;
      PositionPid.error0=change;
      PositionPid.chage=(int)(PositionPid.q0*PositionPid.error0+PositionPid.q1*PositionPid.error1+PositionPid.q2*PositionPid.error2);
      PositionPid.error2=PositionPid.error1;
      PositionPid.error1=PositionPid.error0;
      angle_control=duojmid+PositionPid.chage;
      if(angle_control>duojmax)angle_control=duojmax;
      else if(angle_control<duojmin)angle_control=duojmin;
      PWMDTY67=angle_control-1;
    } 
    if(stop_flag==0&&start_flag!=0)
    {
      speed_return=PulseCnt;
      speed_return0=PulseCnt0; 
      
      if((sum_front1-sum_back1)>rate2&&speed_return>speed2&&speed_return0>speed2)  
      {
        if(backflag==1) 
        {
          PWMDTY4=0;
          PWMDTY1=0;
          asm(nop);
          asm(nop);
          asm(nop);
          PWMDTY5=break_pwm-1;
          PWMDTY0=break_pwm0-1;
        }
        delay(breaktime);
      }else;
 
      PWMDTY5=0;  
      PWMDTY0=0;  
      
      SpeedPid.error0=NowSpeed2-speed_return;
      SpeedPid0.error0=NowSpeed2-speed_return0;
      
      SpeedPid.chage=(int)(SpeedPid.q0*SpeedPid.error0+SpeedPid.q1*SpeedPid.error1+SpeedPid.q2*SpeedPid.error2);
      SpeedPid.error2=SpeedPid.error1;
      SpeedPid.error1=SpeedPid.error0;
      SpeedPid.speed=NowSpeed2+SpeedPid.chage;
       
      SpeedPid0.chage=(int)(SpeedPid0.q0*SpeedPid0.error0+SpeedPid0.q1*SpeedPid0.error1+SpeedPid0.q2*SpeedPid0.error2);
      SpeedPid0.error2=SpeedPid0.error1;
      SpeedPid0.error1=SpeedPid0.error0;
      SpeedPid0.speed=NowSpeed2+SpeedPid0.chage;
       
      if(SpeedPid.speed>NowSpeed2)SpeedPid.speed=NowSpeed2;
      if(SpeedPid0.speed>NowSpeed2)SpeedPid0.speed=NowSpeed2;
      speed_control=SpeedPid.speed;
      speed_control0=SpeedPid0.speed;
      
      PWMDTY1=speed_control0-1;   //恢复PWM输出
      PWMDTY4=speed_control-1;    //恢复PWM输出        
    }
  } 
  else
  {
    if(data[4].d>data[6].d)
    {
      for(;;) 
      { 
        fun_ADC();  
        PWMDTY67=duojmax;
        if(stop_flag==0&&start_flag!=0)
        {
          speed_return=PulseCnt;
          speed_return0=PulseCnt0;      
          
          SpeedPid.error0=NowSpeed3-speed_return;
          SpeedPid0.error0=NowSpeed3-speed_return0;
          
          SpeedPid.chage=(int)(SpeedPid.q0*SpeedPid.error0+SpeedPid.q1*SpeedPid.error1+SpeedPid.q2*SpeedPid.error2);
          SpeedPid.error2=SpeedPid.error1;
          SpeedPid.error1=SpeedPid.error0;
          SpeedPid.speed=NowSpeed3+SpeedPid.chage;
          
          SpeedPid0.chage=(int)(SpeedPid0.q0*SpeedPid0.error0+SpeedPid0.q1*SpeedPid0.error1+SpeedPid0.q2*SpeedPid0.error2);
          SpeedPid0.error2=SpeedPid0.error1;
          SpeedPid0.error1=SpeedPid0.error0;
          SpeedPid0.speed=NowSpeed3+SpeedPid0.chage;
           
          if(SpeedPid0.speed>NowSpeed3)SpeedPid0.speed=NowSpeed3;
          if(SpeedPid.speed>NowSpeed3)SpeedPid.speed=NowSpeed3;
          
          speed_control0=SpeedPid0.speed;
          speed_control=SpeedPid.speed;
          
          PWMDTY1=speed_control-1;    //恢复PWM输出
          PWMDTY4=speed_control0-1;    //恢复PWM输出        
        } 
        if(data[4].d>25)break;
      }
    } 
    if(data[6].d>data[4].d)
    {
      for(;;)
      {
        fun_ADC();
        PWMDTY67=duojmin;
        if(stop_flag==0&&start_flag!=0)
        {
          speed_return=PulseCnt;
          speed_return0=PulseCnt0;      
     
          SpeedPid.error0=NowSpeed3-speed_return;
          SpeedPid0.error0=NowSpeed3-speed_return0;
          
          SpeedPid.chage=(int)(SpeedPid.q0*SpeedPid.error0+SpeedPid.q1*SpeedPid.error1+SpeedPid.q2*SpeedPid.error2);
          SpeedPid.error2=SpeedPid.error1;
          SpeedPid.error1=SpeedPid.error0;
          SpeedPid.speed=NowSpeed3+SpeedPid.chage; 
          
          SpeedPid0.chage=(int)(SpeedPid0.q0*SpeedPid0.error0+SpeedPid0.q1*SpeedPid0.error1+SpeedPid0.q2*SpeedPid0.error2);
          SpeedPid0.error2=SpeedPid0.error1;
          SpeedPid0.error1=SpeedPid0.error0;
          SpeedPid0.speed=NowSpeed3+SpeedPid0.chage; 
          
          if(SpeedPid.speed>NowSpeed3)SpeedPid.speed=NowSpeed3;
          if(SpeedPid0.speed>NowSpeed3)SpeedPid0.speed=NowSpeed3;
          
          speed_control0=SpeedPid0.speed;
          speed_control=SpeedPid.speed;
          
          PWMDTY1=speed_control0-1;    //恢复PWM输出
          PWMDTY4=speed_control-1;    //恢复PWM输出        
        } 
        if(data[6].d>25)break;
      }                                                                   
    } 
  }
}
void Init_cop(void)//看门狗初始化
{         
  COPCTL_WCOP=0;      //正常COP模式
  COPCTL_RSBCK=0;     //在BDM模式下允许COP和RTI运行
  COPCTL_CR2=1;       //CR[2:0]分频值为（2的23次方）
  COPCTL_CR1=1;       //COP溢出周期=OSCCLK/CR[2:0]
  COPCTL_CR0=0;
}
void feed_cop(void)
{
  ARMCOP=0X55;      //在特定时间内依次向ARMCOP写入0X55,0XAA;
  ARMCOP=0XAA;
}
/*********************************************************
                         main()函数d

**********************************************************/
void main(void) 
{
  /* put your own code here */
  DisableInterrupts;     //关总中断
  MCUInit();             //芯片初始化
  IOInit();              //输入输出初始化 
  ADCInit();             //AD初始化
  PWMInit();             //PWM初始化 
  Init_cop();            //看门狗初始化
  speedchoise();
  SpeedPID_Init();       //速度PID数据初始化 
  direction_pid_init();  //舵机PID初始化
  PIT_Init();            // 定时器初始化
  Pulse_int();           //脉冲累加器初始化     
  interrupt_first();     //中断优先级设置
  LCD_Init();
  EnableInterrupts ;     //开总中断
  start_delay();         //起跑延时
  for(;;) 
  {
    if(stop_flag==1) 
    {
      brake_1ms(40);
      start_flag=0;
      stop_flag=0;
    }
    SHOW_TSL8();
    LED_disp();         //调用液晶显示
  }/* loop forever */ 
  /* please make sure that you never leave main */
}
#pragma CODE_SEG __NEAR_SEG NON_BANKED
//------------------------------------------------------------
//PIT0中断函数     //用于测速800*240/48M==4ms
//------------------------------------------------------------
void interrupt 66 PIT0_TSR(void)
{
  DisableInterrupts;
  PITTF_PTF0=1; // 清中断标志位
  feed_cop();   //喂狗程序
  flag=~flag;
  if(flag) 
  {
     PTM=0X00;
     delay(2);
     PulseCnt=PACNT;    //左电机
     PACNT=0 ;                                   
  }
  if(!flag) 
  {  
     PTM=0X02;
     delay(2);
     PulseCnt0=PACNT;    //右电机
     PACNT=0;
  } 
  fun_ADC();
  Counter++;
  Cnt++;
  if(Counter%250==0)    //1秒钟定时增加一次
    select++;        //秒加一
  if(Counter%25==0)
    RD_TSL();
  if(select>5) 
  {           
    if((PORTA_PA0==0)||(PORTA_PA1==0)) 
    {
      stop_flag=1;
    }
  }
  EnableInterrupts;
}
//------------------------------------------------------------
//PIT1中断函数     //用于产生240*1400/48M=7ms基定时
//------------------------------------------------------------
interrupt 67 void PIT1_TSR(void)
{ 
  DisableInterrupts;
  PITTF_PTF1=1;
  direction_control();
  EnableInterrupts;
}

 











