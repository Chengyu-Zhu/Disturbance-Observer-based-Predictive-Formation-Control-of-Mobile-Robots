#include <nav_msgs/Odometry.h>
#include "string.h"
#include "math.h"
#include <iostream>
#include "unistd.h"
#include <sys/time.h>
//#include<conio.h>
#include "wit_motion_sdk.h"
#include "serial/serial.h"
#include <thread>
#include <ros/ros.h>
//extern     serial::Serial Imu_Serial;
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void DelayMs(uint16_t ms);
void ComRxCallBack(char *p_data, uint32_t uiSize);
void sensor_Init();

serial::Serial Imu_Serial;
static char s_cDataUpdate = 0;

void sensor_Init()
{
	try
	{
		Imu_Serial.setPort("/dev/ttyUSB1");
		Imu_Serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		Imu_Serial.setTimeout(to);
		Imu_Serial.open();
	}
	catch (serial::IOException& e)
	{
		std::cout << "Unable to open port" << std::endl;
		std::exit(-1);
	}

	if (Imu_Serial.isOpen())
	{
		std::cout << "Serial Port initialized" << std::endl;
	}
	else
	{
		std::exit(-1);
	}
	AutoScanSensor();
	std::cout << "Serial Port initialized" << std::endl;
}



void ComRxCallBack(char *p_data, uint32_t uiSize)
{
	for(uint32_t i = 0; i < uiSize; i++)
	{
		WitSerialDataIn(p_data[i]);
	}
}


static void DelayMs(uint16_t ms)
{
	ms *= 1000;
	usleep(ms);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Imu_Serial.write((const uint8_t *)p_data, uiSize);
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	s_cDataUpdate = 1;
}

static void AutoScanSensor(void)
{
	WitInit(WIT_PROTOCOL_905x_MODBUS, 0x50);
	std::cout<<"hh"<<std::endl;
	if(WitSerialWriteRegister(SensorUartSend)==WIT_HAL_OK){
		
	}else{
		return;
	}

	if(WitRegisterCallBack(CopeSensorData)==WIT_HAL_OK){
		
	}else{
		return;
	}

	if(WitDelayMsRegister(DelayMs)==WIT_HAL_OK){
		
	}else{
		return;
	}
	usleep(2000);
	const uint32_t c_uiBaud= 115200;
	int i, iRetry;
	unsigned char cBuff[1];
	iRetry = 2;
	s_cDataUpdate = 0;
	do
	{
		WitReadReg(AX, 3);
		usleep(200000);
		while(Imu_Serial.read(cBuff,1))
		{
			WitSerialDataIn(cBuff[0]);
		}
		
		if(s_cDataUpdate != 0)
		{
			printf("%d baud find sensor\r\n\r\n", c_uiBaud);
			return ;
		}
		iRetry--;
	}while(iRetry);			
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odomimu",1);
    nav_msgs::Odometry odom;
    ros::Rate loop_rate(50);
	sensor_Init();
	float a[3], w[3], Angle[3], h[3], Temp;
	int iBuff;
	unsigned char cBuff[37];
    while (ros::ok())
    {
        //gettimeofday(&start_time,NULL);
        WitReadReg(AX, 16);

        Imu_Serial.read(cBuff,37);
        for (int i = 0;i<37;i++)
        {
            WitSerialDataIn(cBuff[i]);
        }
        for (int i = 0;i<3;i++)
        {
        	a[i] = (float)sReg[AX+i]/32768.0f*16.0f;
        	w[i] = (float)sReg[GX+i]/32768.0f*2000.0f;
            
        	iBuff = (((uint32_t)sReg[HRoll + 2 * i]) << 16) | ((uint16_t)sReg[LRoll + 2 * i]);
        	Angle[i] = (float)iBuff / 1000.0f;
        	h[i] = (float)sReg[HX+i];
        }
        iBuff = (float)sReg[Yaw]/32768.0f*180.0f;
        Angle[0] = (float)iBuff;
        Temp = (float)sReg[TEMP905x] / 100.0f;
        //gettimeofday(&end_time,NULL);
        // if(end_time.tv_sec == start_time.tv_sec){
        //     time_diff = end_time.tv_usec - start_time.tv_usec;
        // }
        // else{
        //     time_diff = end_time.tv_usec - start_time.tv_usec + 1000000;
        // }
        // std::cout<<time_diff<<std::endl;
        printf("a:%.2f %.2f %.2f\r\n",a[0],a[1],a[2]);
        printf("w:%.2f %.2f %.2f\r\n",w[0],w[1],w[2]);
        printf("Angle:%.1f %.1f %.1f\r\n",Angle[0],Angle[1],Angle[2]);
        printf("h:%.0f %.0f %.0f\r\n",h[0],h[1],h[2]);
        printf("Temp:%.1f\r\n\r\n", Temp);
        odom.pose.pose.position.z = Angle[0];
        pub.publish(odom);

    }

	return 0;
}