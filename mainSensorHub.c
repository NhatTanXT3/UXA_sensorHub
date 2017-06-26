/*
 * main.c
 * Readversion
 * Test
 */
#include <stdint.h>
#include <stdbool.h>
//#include <math.h>
//#include <stdlib.h>




#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "utils/ringbuf.h"

#include "mySerial.h"

#include "myFIFO.h"
#include "myTimer.h"
#include "myIO.h"
#include "myRS485.h"
#include "SAM.h"

#include "numManipulate.h"

#include "mpu6050.h"
#include "myI2C.h"
#include "IMU.h"
#include "ZMP.h"

void task_200Hz();
void task_100Hz();
void task_50Hz();
void task_20Hz();
void communication();
void display_com();
void task_IMU();
void ZMP_left_update();
void ZMP_right_update();
struct FlagType{
	unsigned char display:1;
}Flag;


void main(){
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //40Mhz
	myIO_init();

	led(LED_RED,0);
	led(LED_BLUE,0);
	led(LED_GREEN,0);

	led(LED_BLUE,1);


	Timer0_init();
	Timer1_init();
	SysTick_Init();

	UART5_Init();
	//	UART7_Init();

	UART1_Init();
	UART6_Init();
	//========sensor init========
	I2C1_Init();
	while(MPU6050_Init());

	SerialPutStrLn(UART_PC_,"calib gyroscope offset ...");
	while(Calib_Gyro());
	//	SerialPutStrLn(UART_PC_,"calib accelerometer offset ...");
	//	while(Calib_Accelerometer_Amplitude());

	//	RS485_4_Init();
	//	RS485_2_Init();

	SerialPutStrLn(UART_PC_,"config done!");
	led(LED_BLUE,0);
	SysCtlDelay(SysCtlClockGet()/300);
	Flag.display=0;


	while(1)
	{
		communication();
		ZMP_left_update();
		ZMP_right_update();

		task_IMU();
		task_20Hz();
		task_50Hz();
		task_100Hz();
				task_200Hz();
	}
}
void task_20Hz(){
	if(FlagTimer.Hz_20)
	{
		FlagTimer.Hz_20=0;
		/*
		 * Your code begin from here
		 */
	}


}

void task_100Hz(){
	if(FlagTimer.Hz_100)
	{
		FlagTimer.Hz_100=0;
		/*
		 * Your code begin from here
		 */

	}
}

void task_200Hz()
{
	if(FlagTimer.Hz_200)
	{
		FlagTimer.Hz_200=0;
		/*
		 * Your code begin from here
		 */
		if(Flag.display)
		{
			getRawZMPData(UART_ZMP_LEFT_);
			getRawZMPData(UART_ZMP_RIGHT_);
		}
	}
}
void task_50Hz(){
	if(FlagTimer.Hz_50)
	{
		FlagTimer.Hz_50=0;
		/*
		 * Your code begin from here
		 */
		if(Flag.display)
		{
			display_com();
			toggle_led[1]^=1;
			led(LED_BLUE,toggle_led[1]);

		}


	}
}

void task_IMU()
{
	if(flag_MPU6050_INTpin==1)
	{
		flag_MPU6050_INTpin=0;
		/*
		 * Your code begin from here
		 */
		uint32_t microSecond=0;
		microSecond=getMicroSecond();
		sampling_time_second=(float)(microSecond- preMicroSecond_angle)/1000000.0;
		preMicroSecond_angle=microSecond;
		/*
		 * update sensor's datas
		 */


		while(MPU6050DataGetRaw(&MPU6050.accX_raw))
		{}
		//		{led(LED_BLUE,1);}// loop untill data is read
		//		led(LED_BLUE,0);

		/*
		 * sensor processing
		 */
		angle(sampling_time_second);
		//		update_accelerometer();
		/*
		 *  running controller
		 */
		//		if(Flag.run_controller)
		//			Stable_Controller(sampling_time_second);
	}// end of (flag_MPU6050_INTpin==1)
}


void communication(){
	if(serialPC.Flag_receive){
		serialPC.Flag_receive=0;
		//=========== your code begin from here========
		switch(serialPC.Command_Data[1]){
		case COM2CTL_DISPLAY_ON_:
			Flag.display=1;
			//			display_mode=Uart.Command_Data[1];
			break;
		case COM2CTL_DISPLAY_OFF_:
			Flag.display=0;
			break;
		default:
			break;
		}
	}
}

void ZMP_left_update(){
	if(serialZMPLeft.Flag_receive){
		serialZMPLeft.Flag_receive=0;
		//=========== your code begin from here========
		if(serialZMPLeft.Command_Data[1]==SENDING_KEY_LEFT_){
			if(((serialZMPLeft.dataIndex-3)%4)==0){ // check length of data
				unsigned char numOfPack=serialZMPLeft.dataIndex/4;
				unsigned char i;
				unsigned char refIndex;
				for(i=0;i<numOfPack;i++)
				{
					refIndex=4*i+2;
					if(((serialZMPLeft.Command_Data[refIndex]^serialZMPLeft.Command_Data[refIndex+1]^serialZMPLeft.Command_Data[refIndex+2])&0x7F)==serialZMPLeft.Command_Data[refIndex+3])
					{
						zmpLeft.rawForceSensor[serialZMPLeft.Command_Data[refIndex]]=(serialZMPLeft.Command_Data[refIndex+1]<<7)+serialZMPLeft.Command_Data[refIndex+2];
					}
					else
					{
						UARTCharPut(UART_PC_,i+48);
						SerialPutStrLn(UART_PC_,"e_zmp2");
					}
				}
			}
			else{
				SerialPutStrLn(UART_PC_,"e_zmp_1");
			}
		}
		else
		{
			SerialPutStrLn(UART_PC_,"e_zmp_0");
		}
	}
}

void ZMP_right_update(){
	if(serialZMPRight.Flag_receive){
		serialZMPRight.Flag_receive=0;
		//=========== your code begin from here========
		if(serialZMPRight.Command_Data[1]==SENDING_KEY_RIGHT_){
			if(((serialZMPRight.dataIndex-3)%4)==0){ // check length of data
				unsigned char numOfPack=serialZMPRight.dataIndex/4;
				unsigned char i;
				unsigned char refIndex;
				for(i=0;i<numOfPack;i++)
				{
					refIndex=4*i+2;
					if(((serialZMPRight.Command_Data[refIndex]^serialZMPRight.Command_Data[refIndex+1]^serialZMPRight.Command_Data[refIndex+2])&0x7F)==serialZMPRight.Command_Data[refIndex+3])
					{
						zmpRight.rawForceSensor[serialZMPRight.Command_Data[refIndex]]=(serialZMPRight.Command_Data[refIndex+1]<<7)+serialZMPRight.Command_Data[refIndex+2];
					}
					else
					{
						UARTCharPut(UART_PC_,i+48);
						SerialPutStrLn(UART_PC_,"e_zmp2");
					}
				}
			}
			else{
				SerialPutStrLn(UART_PC_,"e_zmp_1");
			}
		}
		else
		{
			SerialPutStrLn(UART_PC_,"e_zmp_0");
		}
	}
}

unsigned char display_mode=COM2CTL_DISPLAY_MODE_1_;

void display_com(){
	char buffer[20];

	/*
	 * khao sat PIDx
	 */
	//	SerialPutChar(UART_PC_ ,PC2MCU_HEADER_);
	float2str(IMU.pitch,buffer);
	SerialPutChar(UART_PC_ ,CN_1_);
	SerialPutStr_NonTer(UART_PC_,buffer);

	float2str(IMU.roll,buffer);
	SerialPutChar(UART_PC_ ,CN_2_);
	SerialPutStr_NonTer(UART_PC_,buffer);
	//
	float2str(zmpLeft.rawForceSensor[0],buffer);
	SerialPutChar(UART_PC_ ,CN_3_);
	SerialPutStr_NonTer(UART_PC_,buffer);
	//	//Dte
	float2str(zmpLeft.rawForceSensor[1],buffer);
	SerialPutChar(UART_PC_ ,CN_4_);
	SerialPutStr_NonTer(UART_PC_,buffer);
	//		//
	float2str(zmpLeft.rawForceSensor[2],buffer); //acc_term
	SerialPutChar(UART_PC_ ,CN_5_);
	SerialPutStr_NonTer(UART_PC_,buffer);

	float2str(zmpLeft.rawForceSensor[3],buffer);
	SerialPutChar(UART_PC_ ,CN_6_);
	SerialPutStr_NonTer(UART_PC_,buffer);
	//

	//	float2str(zmpRight.rawForceSensor[0],buffer);
	//	SerialPutChar(UART_PC_ ,CN_7_);
	//	SerialPutStr_NonTer(UART_PC_,buffer);
	//
	//	float2str(zmpRight.rawForceSensor[1],buffer);
	//	SerialPutChar(UART_PC_ ,CN_8_);
	//	SerialPutStr_NonTer(UART_PC_,buffer);
	//	//
	//	float2str(zmpRight.rawForceSensor[2],buffer);
	//	SerialPutChar(UART_PC_ ,CN_9_);
	//	SerialPutStr_NonTer(UART_PC_,buffer);
	//
	//	float2str(zmpRight.rawForceSensor[3],buffer);
	//	SerialPutChar(UART_PC_ ,CN_10_);
	//	SerialPutStrLn(UART_PC_,buffer);
	//SerialTerminator(UART_PC_);
	//	SerialPutChar(UART_PC_ ,PC2MCU_TERMINATOR_);
	SerialTerminator(UART_PC_);
}


