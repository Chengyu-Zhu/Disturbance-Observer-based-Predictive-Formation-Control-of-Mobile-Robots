

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <math.h>

#include "controlcan.h"

#define DEVICE_ID 0x602

//can 0 是右    can1是左


static VCI_BOARD_INFO pInfo[50];//用来获取设备信息。
static int num = 0;
static std::mutex m;
static std::mutex m_init;



class motor_init
{

    public:
        motor_init();//打开can设备，初始化电机，使能电机
        ~motor_init();//失能电机，关闭can设备

        /**
         * 电机使能函数
         */
        void Motor_Enable();

        /**
         * 电机失能函数
         */
        void Motor_Disable();

        /**
         * 设置电机速度
         * @param [in] leftValue  电机左轮速度（串口）
         * @param [in] rightValue 电机右轮速度（串口）  
         */
        void Set_Velocity(double leftValue,double rightValue);
        
        /**
         * 获取速度指令
         */
        void Get_Velocity();

    
        
    private:

        /**
         * 打开can设备
         */
        void open_usbcan();

        /**
         * 关闭can设备
         */
        void close_usbcan();

        /**
         * 电机初始化函数
         */
        void Motor_Init();

        void Send_Msg_init(PVCI_CAN_OBJ Send_data, uint n);
        void Send_Msg(PVCI_CAN_OBJ pSend, DWORD DeviceType, uint n);

        double Rotation_speed_Limit(double value);

        double maxRotation_speed = 15000; // 转速限制 串口   对应  m/s
        


};






