#include "stm32f10x.h"
#include "stdbool.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "spi.h"
#include "usart.h"
#include "timer.h"
#include "gpio.h"
#include "exti.h"
#include "stmflash.h"

#define FLASH_SAVE_ADDR  0X0807F000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

#define   OPEN 				1
#define   CLOSE       0
#define   LOCK_OPEN   {PCout(2) = 1;}
#define   LOCK_CLOSE   {PCout(2) = 0;}	
#define   Motor_RUN   {PCout(3) = 1;}
#define   Motor_STOP   {PCout(3) = 0;}

uint8_t x_start = 10;
bool Flag_0WiFi_1ZigBee = 0;
bool Flag_car_status = CLOSE;
uint32_t Value_Accumulated_Mileage_current_order = 0;
uint16_t Light_threshold = 200;          // 光照阈值，默认为200 Lux，可通过上下键调节
bool Flag_manual_light_mode = 0;         //手动灯光模式标志，0=自动，1=手动
uint32_t Value_Accumulated_Mileage_total = 0;
unsigned char Read_Humiture_CMD[8]		={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};//读取温湿度数据的命令
unsigned char Read_Illuminance_CMD[8] = {0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};  //读取光照传感器的 Modbus 命令
char Str_Light_threshold[4] = {'5','0','0',0};
char Str_RTC_time[9] = {'0','0',':','0','0',':','0','0',0};
char Light_value_str[7] = {0};
unsigned long Light_value = 0, temp_u32 = 0;   //当前光照值（Lux）
bool Flag_LED_ONOFF = 0;
bool Flag_ON_OFF_left_light = 0;
bool Flag_ON_OFF_right_light = 0;
uint8_t Flag_type_Low_High_close_light = 0;
bool Flag_auto_light = 0;
char Data_buf_WiFi_Light[17] = {"Hwd04#IL:000000T"};
char Str_Value_Accumulated_Mileage_current_order[6] = {'0','0','.','0','0',0};
char Str_Value_Accumulated_Mileage_total[7] = {0};
uint16_t Buf_Save_Flash[3] = {0};
bool Flag_LED3_ONOFF = 0;
bool Flag_blink_ONOFF = 0;
unsigned int Tem_value = 0,Hum_value = 0;
uint16_t Value_threshold 																	= 300;//阈值，默认为30.0℃
bool Flag_Alarm = 0;//报警标志置一
bool Flag_allow_alarm = 1;//允许累加报警次数标志是否置一
//uint16_t Count_times_Alarm = 0;
//char Str_Count_times_Alarm[4] = {'0','0','0',0};
char Str_Value_threshold[5] = {'0','0','.','0',0};
char Temp_value_str[7];
char Hum_value_str[6];
char Temp_max_value_str[5] = {0};
uint16_t Max_temperature = 0;
uint8_t Max_voltage = 0;
uint8_t Count_Value_Accumulated_temes_alarm = 0;
char Str_Count_Value_Accumulated_temes_alarm[4] = {0};
uint16_t Accumulated_Mileage_total = 0;
char Data_buf_WiFi[50];
char Data_buf_ZigBee[70];
unsigned char Flag_Sonser_Device_onoff = 0;
char AT_send_buf[20];
unsigned char x,flag = 1;
void copy_str(char* des,char* src,unsigned char len);
void wait_OK(void);
void wait_dayuhao(void);
unsigned char Get_data_len(char* addr);//获取字符串/数组有效数据长度
void Wifi_send(char* Data_buf);
void ESP8266_Init(void);
void LCD_display_init(void);
unsigned char Query(char * buf,char * str,unsigned int LEN);
void Regularly_collect_data(void);//定期发送采集传感器数据的命令
void Usart1_receive_process(void);//串口1数据接收处理
void Usart3_receive_process(void);//串口3数据接收处理
void Key_procsess(void);//按键处理
void Mileage_process(void);//里程数处理
void Renew_Flash(void);//更新存入FLASH里面的数据，包括车辆累计总里程、历史温度过高警报次数
void FLASH_data_init(void);//FLASH初始化
void Patrol_inspection_mode_process(void);//巡检模式处理
void Light_Control(void);
int main(void)
{
	GPIO_Configuration();				//锁和电机的GPIO初始化
	delay_init();	    	 			//延时函数初始化	
	Usart1_Init(115200);			//串口1初始化
	Usart2_Init(115200);			//串口2初始化
	Usart3_Remap_Init(9600);    //温度串口3重映射接口初始化 
	NVIC_Configuration(); 		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级 	LED_Init();			     //LED端口初始化	
	EXTIX_Init();//外部中断初始化
	TIM3_Int_Init(999,7199);	//开启定时器3，计数100ms
	for(x = 0;x < 20;x ++)
		AT_send_buf[x] = '\0';
	Lcd_Init();					//初始化LCD
	FLASH_data_init();//FLASH初始化	
	ESP8266_Init();//初始化WIFI模块
	USART1TxStr("Hwd02connectT");//透传方式发送初始握手数据
	LCD_display_init();//LCD初始化显示画面	
	copy_str(Data_buf_WiFi,"Hwd02#TE:00.0#VO:55#LA:0#KM:00.00T",34);
	copy_str(Data_buf_ZigBee,"Hzd02#TE:00.0,UP:00.0#VO:00,UP:00#LA:0,ADD:000#KM:00.00,ADD:000.00T",67);
	while(1)
	{
		Regularly_collect_data();//定期发送采集传感器数据的命令
		Usart1_receive_process();//串口1-WiFi或ZigBee数据接收处理
		Usart3_receive_process();//串口3-传感器数据接收处理
		Key_procsess();//按键处理
		Light_Control();              // 自动灯光控制：根据时间或光照阈值开关车灯
		Mileage_process();//里程数累加处理	
	}
}
void FLASH_data_init(void)//FLASH初始化
{
	if(KEY_down == 0)//如果是按着下按键进行设备开机或重启，则对FLASH内存放的用户数据进行清零操作
	{
		KEY_value = key_free;
		STMFLASH_Write(FLASH_SAVE_ADDR,(unsigned int*)Buf_Save_Flash,3);
		delay_ms(1000);
		BACK_COLOR=BLACK;		//背景为黑色
		POINT_COLOR=YELLOW;	//画笔为黄色
		LCD_Show_Chinese16x16(10,30,"      数据已清零");
		LCD_ShowString(10,30,"FLASH");
	}	
	STMFLASH_Read(FLASH_SAVE_ADDR,(unsigned int*)Buf_Save_Flash,3);//读取存放在FLASH的用户数据
	Count_Value_Accumulated_temes_alarm = Buf_Save_Flash[1] & 0x00FF; //取出累计的温度过高警报次数
	Value_Accumulated_Mileage_total = Buf_Save_Flash[2];//取出累计的历史里程
	
	Renew_Flash();
	
	Str_Value_Accumulated_Mileage_total[0] = Value_Accumulated_Mileage_total % 1000000 / 100000 + '0';
	Str_Value_Accumulated_Mileage_total[1] = Value_Accumulated_Mileage_total % 100000 / 10000 + '0';
	Str_Value_Accumulated_Mileage_total[2] = Value_Accumulated_Mileage_total % 10000 / 1000 + '0';
	Str_Value_Accumulated_Mileage_total[3] = '.';
	Str_Value_Accumulated_Mileage_total[4] = Value_Accumulated_Mileage_total % 1000 / 100 + '0';
	Str_Value_Accumulated_Mileage_total[5] = Value_Accumulated_Mileage_total % 100 / 10 + '0';
			
	
  
}
void Usart1_receive_process(void)//串口1数据接收处理
{
	if(Flag_Usart1_Receive//如果串口1有数据接收
			&&(!Count_Timer3_value_USART1_receive_timeout))//并且如果接收超时
	{
		Flag_Usart1_Receive = 0;
		if(Query(USART1_RX_BUF,"HwcstartT",USART1_REC_LEN))
		{
			Flag_car_status = OPEN;
			LOCK_OPEN;//打开锁
			POINT_COLOR = YELLOW;//切换黄色
			LCD_Show_Chinese16x16(x_start+16*11,10+3*2+16*2,"开启");
			Value_Accumulated_Mileage_current_order = 0;//当前订单里程清零
			LCD_ShowString(x_start+16*11,10+3*4+16*4,"00.00 KM");//当前订单累计里程（重新刷卡启动后清零）	
			Buzzer_ON();
			delay_ms(20);
			Buzzer_OFF();
			delay_ms(200);
			Buzzer_ON();
			delay_ms(20);
			Buzzer_OFF();
			CLR_Buf1();//清除串口1接收缓存			
		}
		if(Query(USART1_RX_BUF,"HwcstopT",USART1_REC_LEN))
		{
			Buzzer_ON();
			delay_ms(200);
			Buzzer_OFF();
			Flag_car_status = CLOSE;
			LOCK_CLOSE;//闭合锁
			POINT_COLOR = YELLOW;//切换黄色
			LCD_Show_Chinese16x16(x_start+16*11,10+3*2+16*2,"关闭");				
			Motor_STOP;//关闭电机
			Flag_state_Accumulated_Mileage = CLOSE;//停止里程累计
			Renew_Flash();//更新存入FLASH里面的数据，包括车辆累计总里程、历史温度过高警报次数	
			CLR_Buf1();//清除串口1接收缓存			
		}	
		if(Flag_0WiFi_1ZigBee == 1)
		{
			if(Query(USART1_RX_BUF,"Hzc01#getdataT",USART1_REC_LEN))//巡检设备发来的获取数据请求
			{			
				USART2TxStr("收到请求，准备返回数据");//向巡检设备发送车辆信息
				//需要回传的数据（67个字节）：Hzd02#TE:00.0,UP:00.0#VO:00,UP:00#LA:0,ADD:000#KM:00.00,ADD:000.00T
				copy_str(&Data_buf_ZigBee[9],&Temp_value_str[2],4);//嵌入当前温度值
				copy_str(&Data_buf_ZigBee[17],Temp_max_value_str,4);//嵌入最高温度值
				copy_str(&Data_buf_ZigBee[25],"55",2);//嵌入电池当前电压值
				copy_str(&Data_buf_ZigBee[31],"72",2);//嵌入电池最高电压值
				if(Flag_Alarm)
					copy_str(&Data_buf_ZigBee[37],"1",1);//嵌入当前是否有温度警报
				else
					copy_str(&Data_buf_ZigBee[37],"0",1);//嵌入当前是否有温度警报
				
				copy_str(&Data_buf_ZigBee[43],Str_Count_Value_Accumulated_temes_alarm,3);//嵌入历史温度警报次数
				copy_str(&Data_buf_ZigBee[50],Str_Value_Accumulated_Mileage_current_order,5);//嵌入当前订单已行驶里程数
				copy_str(&Data_buf_ZigBee[60],Str_Value_Accumulated_Mileage_total,6);//嵌入历史累计里程数
				Data_buf_ZigBee[66] = 'T';
				Data_buf_ZigBee[67] = 0;
				USART1TxStr(Data_buf_ZigBee);//向巡检设备发送车辆信息
				
				USART2TxStr("通过ZigBee发送数据：");//向巡检设备发送车辆信息
				USART2TxStr(Data_buf_ZigBee);//
				CLR_Buf1();
			}
		}
		POINT_COLOR=GREEN;//切换绿色
	}
}
void Usart3_receive_process(void)
{
	//接收光照传感器数据，解析出 Light_value（单位：Lux）
	if(Flag_Usart3_Receive && (!Count_Timer3_value_USART3_receive_timeout))
	{
		Flag_Usart3_Receive = 0;
		Light_value = USART3_RX_BUF[4];
		Light_value <<= 16;
		temp_u32 = USART3_RX_BUF[5];
		temp_u32 <<= 8;
		Light_value |= temp_u32;
		Light_value |= USART3_RX_BUF[6];
		CLR_Buf3();

		// 将光照值转换为字符串用于显示
		Light_value_str[0] = (char)(Light_value / 100000 + '0');
		Light_value_str[1] = (char)(Light_value % 100000 / 10000 + '0');
		Light_value_str[2] = (char)(Light_value % 10000 / 1000 + '0');
		Light_value_str[3] = (char)(Light_value % 1000 / 100 + '0');
		Light_value_str[4] = (char)(Light_value % 100 / 10 + '0');
		Light_value_str[5] = (char)(Light_value % 10 + '0');
		Light_value_str[6] = 0;

		POINT_COLOR = YELLOW;
		LCD_ShowString(x_start+16*11,10+3*4+16*4, Light_value_str);
		copy_str(&Data_buf_WiFi_Light[9], Light_value_str, 6);
		POINT_COLOR = GREEN;
	}
}
void Key_procsess(void)
{
	//按键功能：
	// - 右键：切换手动/自动灯光模式
	// - 上键/下键：当车锁关闭时，用于调节光照阈值（步长10，范围0~999）
	// - 左键：切换WiFi/ZigBee（非核心功能）
	// 注意：车锁开启时，上下键控制电机（非灯光功能，但不会开锁）
	if(KEY_value != key_free)
	{
		if(Flag_car_status == OPEN)
		{
			// 车锁开启时，上下键控制电机（与灯光无关，忽略）
			if(KEY_value == up)
			{
				KEY_value = key_free;
				Motor_RUN;
				Flag_state_Accumulated_Mileage = OPEN;
			}
			if(KEY_value == down)
			{
				KEY_value = key_free;
				Motor_STOP;
				Flag_state_Accumulated_Mileage = CLOSE;
				Renew_Flash();
			}
		}
		else
		{
			//车锁关闭时，上下键调节光照阈值
			if(KEY_value == up)
			{
				KEY_value = key_free;
				Light_threshold += 10;
				if(Light_threshold > 999) Light_threshold = 999;
			}
			if(KEY_value == down)
			{
				KEY_value = key_free;
				if(Light_threshold >= 10) Light_threshold -= 10;
			}
		}
		if(KEY_value == left)
		{
			KEY_value = key_free;
			// 切换WiFi/ZigBee，非灯光核心功能，忽略
			if(Flag_0WiFi_1ZigBee == 0)
			{
				Flag_0WiFi_1ZigBee = 1;
				Usart1_Remap_Init(115200);
				LCD_Show_Chinese16x16(x_start+16*7,10+3*11+16*11, "              ");
				LCD_ShowString(x_start+16*7,10+3*11+16*11, "ZigBee (    )");
				LCD_Show_Chinese16x16(x_start+16*11,10+3*11+16*11, "\xd1\xb2\xbc\xec");
			}
			else if(Flag_0WiFi_1ZigBee == 1)
			{
				Flag_0WiFi_1ZigBee = 0;
				Usart1_Init(115200);
				LCD_Show_Chinese16x16(x_start+16*7,10+3*11+16*11, "              ");
				LCD_ShowString(x_start+16*7,10+3*11+16*11, "WiFi (      )");
				LCD_Show_Chinese16x16(x_start+16*10,10+3*11+16*11, "\xc1\xac\xbd\xd3\xd6\xd0");
			}
		}
		if(KEY_value == right)
		{
			// 右键：切换手动/自动灯光模式，并立即执行对应操作
			KEY_value = key_free;
			Flag_manual_light_mode = !Flag_manual_light_mode;
			if(Flag_manual_light_mode)
			{
				LED1_ON();
				LED2_ON();
				Flag_LED_ONOFF = 1;
			}
			else
			{
				LED1_OFF();
				LED2_OFF();
				Flag_LED_ONOFF = 0;
			}
		}
		KEY_value = key_free;
	}
}
void Renew_Flash(void)//更新存入FLASH里面的数据，包括车辆累计总里程、历史温度过高警报次数
{
	/*要存储的数据内容举例（16位数据）：
	Buf_Save_Flash[0]：	存放历史最高温度，数据举例：802 （80.2度）；
	Buf_Save_Flash[1]：	高8位存放历史最高电压，数据举例：70 （70V）；
											低8位存放历史累计温度过高警报次数，数据举例：123（123）次；
	Buf_Save_Flash[2]：	存放车辆历史总累计里程，数据举例：12345 （123.45KM）。
	
	变量名：
	Max_temperature：最高温度
	Max_voltage：最高电压
	Accumulated_temes_number_alarm：累计温度过高警报次数
	Accumulated_Mileage_total：总累计里程
	*/
	uint16_t temp_Buf[3] = {0};	
	if(Buf_Save_Flash[0] < Max_temperature)
	{
		temp_Buf[0] = Max_temperature;	
	}
	else
	{
		temp_Buf[0] = Buf_Save_Flash[0];
	}
	temp_Buf[1] =(Max_voltage << 8) + Count_Value_Accumulated_temes_alarm;
	temp_Buf[2] = Value_Accumulated_Mileage_total;
	STMFLASH_Write(FLASH_SAVE_ADDR,(unsigned int*)temp_Buf,3);
	delay_ms(1000);
	STMFLASH_Read(FLASH_SAVE_ADDR,(unsigned int*)Buf_Save_Flash,3);//读取存放在FLASH的用户数据
		
	
	Temp_max_value_str[0]	=	(char)(Buf_Save_Flash[0] % 1000 / 100 + '0');
	Temp_max_value_str[1]	=	(char)(Buf_Save_Flash[0] % 100 / 10 + '0');
	Temp_max_value_str[2]	=	'.';
	Temp_max_value_str[3]	=	(char)(Buf_Save_Flash[0] % 10 + '0');
  
	Str_Value_Accumulated_Mileage_total[0] = Buf_Save_Flash[2] % 1000000 / 100000 + '0';
	Str_Value_Accumulated_Mileage_total[1] = Buf_Save_Flash[2] % 100000 / 10000 + '0';
	Str_Value_Accumulated_Mileage_total[2] = Buf_Save_Flash[2] % 10000 / 1000 + '0';
	Str_Value_Accumulated_Mileage_total[3] = '.';
	Str_Value_Accumulated_Mileage_total[4] = Buf_Save_Flash[2] % 1000 / 100 + '0';
	Str_Value_Accumulated_Mileage_total[5] = Buf_Save_Flash[2] % 100 / 10 + '0';	
  
	Count_Value_Accumulated_temes_alarm = Buf_Save_Flash[1] & 0x00FF; //取出累计的温度过高警报次数
	Str_Count_Value_Accumulated_temes_alarm[0] = Count_Value_Accumulated_temes_alarm % 1000 / 100 + '0';
	Str_Count_Value_Accumulated_temes_alarm[1] = Count_Value_Accumulated_temes_alarm % 100 / 10 + '0';
	Str_Count_Value_Accumulated_temes_alarm[2] = Count_Value_Accumulated_temes_alarm % 10 + '0';
	
	LCD_ShowString(x_start+16*11,10+3*7+16*7,"    ");
//	LCD_ShowString(x_start+16*11,10+3*5+16*5,"      ");
	LCD_ShowString(x_start+16*11,10+3*9+16*9,"   ");
  delay_ms(500);	
	LCD_ShowString(x_start+16*11,10+3*7+16*7,Temp_max_value_str);//历史最高温度
//	LCD_ShowString(x_start+16*11,10+3*5+16*5,Str_Value_Accumulated_Mileage_total);//历史累计里程（掉电保存）	
	LCD_ShowString(x_start+16*11,10+3*9+16*9,Str_Count_Value_Accumulated_temes_alarm);//历史温度过高警报次数
}
void Mileage_process(void)
{
	// 里程累计，非核心功能，忽略
	if(Flag_state_Accumulated_Mileage == OPEN)
	{
		if(Flag_timer_Accumulated_Mileage_1S)
		{
			Flag_timer_Accumulated_Mileage_1S = 0;
			if(Flag_blink_ONOFF)
			{
				Flag_blink_ONOFF = 0;
				LCD_Fill(x_start+16*17,10+3*4+16*4, x_start+16*17+16,10+3*4+16*4+16, YELLOW);
			}
			else
			{
				Flag_blink_ONOFF = 1;
				LCD_Fill(x_start+16*17,10+3*4+16*4, x_start+16*17+16,10+3*4+16*4+16, BLACK);
			}
			Value_Accumulated_Mileage_current_order += 5;
			Value_Accumulated_Mileage_total += 5;
			Str_Value_Accumulated_Mileage_current_order[0] = Value_Accumulated_Mileage_current_order % 100000 / 10000 + '0';
			Str_Value_Accumulated_Mileage_current_order[1] = Value_Accumulated_Mileage_current_order % 10000 / 1000 + '0';
			Str_Value_Accumulated_Mileage_current_order[2] = '.';
			Str_Value_Accumulated_Mileage_current_order[3] = Value_Accumulated_Mileage_current_order % 1000 / 100 + '0';
			Str_Value_Accumulated_Mileage_current_order[4] = Value_Accumulated_Mileage_current_order % 100 / 10 + '0';
			copy_str(&Data_buf_WiFi[28], Str_Value_Accumulated_Mileage_current_order, 5);

			Str_Value_Accumulated_Mileage_total[0] = Value_Accumulated_Mileage_total % 1000000 / 100000 + '0';
			Str_Value_Accumulated_Mileage_total[1] = Value_Accumulated_Mileage_total % 100000 / 10000 + '0';
			Str_Value_Accumulated_Mileage_total[2] = Value_Accumulated_Mileage_total % 10000 / 1000 + '0';
			Str_Value_Accumulated_Mileage_total[3] = '.';
			Str_Value_Accumulated_Mileage_total[4] = Value_Accumulated_Mileage_total % 1000 / 100 + '0';
			Str_Value_Accumulated_Mileage_total[5] = Value_Accumulated_Mileage_total % 100 / 10 + '0';

			POINT_COLOR = YELLOW;
			LCD_ShowString(x_start+16*11,10+3*4+16*4, Str_Value_Accumulated_Mileage_current_order);
			LCD_ShowString(x_start+16*11,10+3*5+16*5, Str_Value_Accumulated_Mileage_total);
		}
	}
}
void Regularly_collect_data(void)
{
	// 每2秒执行一次：读取光照传感器，刷新LCD显示的RTC时间、阈值、灯光模式、灯光状态
	if(Flag_timer_2S)
	{
		Flag_timer_2S = 0;
		USART3TxData_hex(Read_Illuminance_CMD, 8);   // 发送命令读取光照值

		// 以下为指示灯和显示刷新，非核心灯光控制逻辑
		if(Flag_LED3_ONOFF)
		{
			Flag_LED3_ONOFF = 0;
			LED3_ON();
			LCD_Fill(x_start+16*17,10+3*6+16*6, x_start+16*17+16,10+3*6+16*6+16, YELLOW);
		}
		else
		{
			Flag_LED3_ONOFF = 1;
			LED3_OFF();
			LCD_Fill(x_start+16*17,10+3*6+16*6, x_start+16*17+16,10+3*6+16*6+16, BLACK);
		}

		POINT_COLOR = YELLOW;
		LCD_ShowString(x_start+16*11,10+3*3+16*3, Str_RTC_time);
		LCD_ShowString(x_start+16*11,10+3*5+16*5, Str_Light_threshold);
		if(Flag_manual_light_mode)
			LCD_ShowString(x_start+16*11,10+3*6+16*6,"Manual");
		else
			LCD_ShowString(x_start+16*11,10+3*6+16*6,"Auto  ");
		if(Flag_LED_ONOFF)
			LCD_ShowString(x_start+16*16,10+3*6+16*6,"ON ");
		else
			LCD_ShowString(x_start+16*16,10+3*6+16*6,"OFF");
		POINT_COLOR = GREEN;

		if(Flag_car_status == OPEN)
		{
			if(Flag_0WiFi_1ZigBee == 0)
				USART1TxStr(Data_buf_WiFi);
		}
	}
}
void wait_OK(void)
{
	while(!Flag_usart1_receive_OK);//等待接收到OK字符串
	Flag_usart1_receive_OK = 0;
	CLR_Buf1();     //清除串口1接收缓存
}
void wait_dayuhao(void)
{
	while(flag)
	{
		for(x = 0;x < USART1_RX_STA;x ++)
		{
			if(USART1_RX_BUF[x] == '>')
			{//等待收到大于号字符串
				flag = 0;
				break;
			}					
		}
	}
	flag = 1;
}
void copy_str(char* des,char* src,unsigned char len)
{
	unsigned char i;
	for(i = 0;i < len;i ++)
	{
		*(des+i) = *(src+i);
	}
}
unsigned char Get_data_len(char* addr)//获取字符串/数组有效数据长度
{
	unsigned len;
	for(len = 0;*(addr+len) != '\0';len ++);
	return len;
}
void Wifi_send(char* Data_buf)
{
	//通过WIFI发送数据给服务器
	char AT_send_buf[20];
	copy_str(AT_send_buf,"AT+CIPSEND=4,0",14);//拷贝字符串
	x = Get_data_len(Data_buf);//获取字符串/数组有效数据长度
	if(x < 10)
	{
		AT_send_buf[13] = x % 10 + '0';
		AT_send_buf[14] = '\r';
		AT_send_buf[15] = '\n';
	}
	else
	{
		AT_send_buf[13] = x % 100 / 10 + '0';
		AT_send_buf[14] = x % 10  + '0';
		AT_send_buf[15] = '\r';
		AT_send_buf[16] = '\n';
	}
	USART2TxStr(AT_send_buf);
	USART1TxStr(AT_send_buf);//发送字符串长度	
	POINT_COLOR=YELLOW;	//画笔为黄色
	LCD_ShowString(120+16*6,40+18*6,"send len... ");			
	while(flag)
	{
		for(x = 0;x < USART1_RX_STA;x ++)
		{
			if(USART1_RX_BUF[x] == '>')
			{//等待收到大于号字符串
				flag = 0;
				break;
			}					
		}
	}
	flag = 1;
	USART2TxStr(Data_buf);
	USART1TxStr(Data_buf);//发送字符串
	LCD_ShowString(120+16*6,40+18*6,"send data...");
	wait_OK();//等待接收到OK字符串
	LCD_ShowString(120+16*6,40+18*6,"Done !      ");
	CLR_Buf1();     //清除串口1接收缓存
	CLR_Buf2();     //清除串口2接收缓存
	for(x = 0;x < 20;x ++)
		AT_send_buf[x] = 0;
}
void ESP8266_Init(void)
{
	BACK_COLOR=BLACK;		//背景为黑色
	POINT_COLOR=YELLOW;	//画笔为黄色
	LCD_ShowString(0,50,"Connecting wifi...");
	USART1TxStr("+++");//结束当前透传，先发次命令，防止设备重新运行是WiFi模块没重启，还处于透传模式
	delay_ms(500);//值最大为1864
	USART1TxStr("AT+CIPMODE=0\r\n");//关闭透传模式，先发次命令，防止设备重新运行是WiFi模块没重启，还处于透传模式
	delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	//重启WiFi模块
	USART2TxStr("重启模块...\r\n"); 
	USART1TxStr("AT+RST\r\n");
  delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	delay_ms(1000);//值最大为1864
	CLR_Buf1();     //清除串口1接收缓存
	Flag_usart1_receive_OK = 0;
	//设置工作模式
	USART2TxStr("设置工作模式...\r\n"); 
	USART1TxStr("AT+CWMODE=1\r\n");
	LCD_ShowString(0,70,"AT+CWMODE=1");
	wait_OK();//等待接收到OK字符串
	
	//连接已知WiFi
	USART2TxStr("连接已知WiFi...\r\n");	
	USART1TxStr("AT+CWJAP=\"THREE\",\"96N8?51e\"\r\n");
	LCD_ShowString(0,90,"AT+CWJAP=\"THREE\",\"96N8?51e\"");
	wait_OK();//等待接收到OK字符串
	
	//关闭多模块连接
	USART2TxStr("关闭多模块连接...\r\n");
	USART1TxStr("AT+CIPMUX=0\r\n");
	LCD_ShowString(0,110,"AT+CIPMUX=0");
	wait_OK();//等待接收到OK字符串
	
	//连接服务器
	USART2TxStr("连接服务器...\r\n");
	USART1TxStr("AT+CIPSTART=\"TCP\",\"192.168.137.1\",8080\r\n");
	LCD_ShowString(0,130,"AT+CIPSTART=\"TCP\",\"192.168.137.1\",8080");	
//	USART1TxStr("AT+CIPSTART=\"TCP\",\"192.168.1.6\",6000\r\n");
//	LCD_ShowString(0,130,"AT+CIPSTART=\"TCP\",\"192.168.1.6\",6000");	
	wait_OK();//等待接收到OK字符串
	
  USART1TxStr("AT+CIPMODE=1\r\n");//开启透传模式
	wait_OK();//等待接收到OK字符串
	USART1TxStr("AT+CIPSEND\r\n");//开始透传
	wait_dayuhao();//等待返回大于号
}

void Light_Control(void)
{
	// 自动灯光控制逻辑
	// 条件：手动模式关闭时，若（时间 >= 19点）或（光照值 < 阈值），则开灯；否则关灯
	if(Flag_manual_light_mode == 0)   // 自动模式
	{
		// rtc_hour 来自 timer.c 中的 RTC 软件时钟
		if((rtc_hour >= 19) || (Light_value < Light_threshold))
		{
			LED1_ON();
			LED2_ON();
			Flag_LED_ONOFF = 1;
		}
		else
		{
			LED1_OFF();
			LED2_OFF();
			Flag_LED_ONOFF = 0;
		}
	}
	// 手动模式下，灯光状态已在按键处理中直接控制，此处不再改变

	// 更新显示字符串（阈值、RTC时间）
	Str_Light_threshold[0] = Light_threshold / 100 + '0';
	Str_Light_threshold[1] = Light_threshold % 100 / 10 + '0';
	Str_Light_threshold[2] = Light_threshold % 10 + '0';

	Str_RTC_time[0] = rtc_hour / 10 + '0';
	Str_RTC_time[1] = rtc_hour % 10 + '0';
	Str_RTC_time[3] = rtc_min / 10 + '0';
	Str_RTC_time[4] = rtc_min % 10 + '0';
	Str_RTC_time[6] = rtc_sec / 10 + '0';
	Str_RTC_time[7] = rtc_sec % 10 + '0';
}

void LCD_display_init(void)
{
	// LCD 初始化显示界面，非控制逻辑，仅显示相关
	LCD_Clear(BLACK);
	POINT_COLOR = GREEN;
	LCD_DrawRectangle(0, 0, 320, 240);
	LCD_ShowString(10,10,"Three");
	LCD_ShowString(60,10+3*1+16*0,"Shared Bicycle System");
	LCD_Show_Chinese16x16(x_start+16*5,10+3*2+16*2, "\xb3\xb5\xcb\xf8\xd7\xb4\xcc\xac\xa3\xba\xb9\xd8\xb1\xd5");
	LCD_ShowString(x_start+16*5,10+3*3+16*3,"RTC Time:");
	LCD_ShowString(x_start+16*11,10+3*3+16*3,"00:00:00");
	LCD_ShowString(x_start+16*5,10+3*4+16*4,"light(Lux):");
	LCD_ShowString(x_start+16*11,10+3*4+16*4,"000000");
	LCD_ShowString(x_start+16*5,10+3*5+16*5,"Threshold:");
	LCD_ShowString(x_start+16*11,10+3*5+16*5,"200");
	LCD_ShowString(x_start+16*5,10+3*6+16*6,"Lighting Mode :");
	LCD_ShowString(x_start+16*11,10+3*6+16*6,"Auto ");
	LCD_ShowString(x_start+16*16,10+3*6+16*6,"OFF");
	LCD_Show_Chinese16x16(x_start+16*0,10+3*11+16*11,"\xb5\xb1\xc7\xb0\xcd\xa8\xd0\xc5\xb7\xbd\xca\xbd\xa3\xba      \xc1\xac\xbd\xd3\xd6\xd0  \xc7\xd0\xbb\xbb");
	LCD_ShowString(x_start+16*7,10+3*11+16*11, "WiFi (");
	LCD_ShowString(x_start+16*13,10+3*11+16*11, ")");

	Str_Value_Accumulated_Mileage_total[0] = Value_Accumulated_Mileage_total % 1000000 / 100000 + '0';
	Str_Value_Accumulated_Mileage_total[1] = Value_Accumulated_Mileage_total % 100000 / 10000 + '0';
	Str_Value_Accumulated_Mileage_total[2] = Value_Accumulated_Mileage_total % 10000 / 1000 + '0';
	Str_Value_Accumulated_Mileage_total[3] = '.';
	Str_Value_Accumulated_Mileage_total[4] = Value_Accumulated_Mileage_total % 1000 / 100 + '0';
	Str_Value_Accumulated_Mileage_total[5] = Value_Accumulated_Mileage_total % 100 / 10 + '0';
	LCD_ShowString(x_start+16*11,10+3*7+16*7, Str_Value_Accumulated_Mileage_total);
}
unsigned char Query(char * buf,char * str,unsigned int LEN)
	//查询数组有无包含该字符串，有则返回1，无则返回0
{
	unsigned int y= 0,len= 0,n= 0;
	unsigned char Result = 0;
	char * i;
	i = str;
	for(; *i != '\0';i ++,len ++){}// 判断需要检测的字符的长度
	for(y = 0; y < LEN - len;y ++)
		//开始检测，次数为总长度减去字符长度的字节数
	{
		for(n = 0;n < len;n ++)
		{
			if(*(buf + y + n) == *(str + n))
				//开始检测双方的第一个字节，如果相等则结果等于1，并且继续检测双方的第二个字节
			{
				Result = 1;				
			}
			else
			{
				Result = 0;	//不相等则结果等于0，并且退出此次循环，
										//开始检测数组的第二个字节和字符的第一个字节
				break;
			}
		}
		if(n == len)
		{
			return Result;
		}
	}
	return Result;
}
