#pragma once
//串口头文件
#include "serial/serial.h"
//c++
#include <iostream>
#include <fstream>
//多线程头文件
#include <mutex>
#include <thread>
//数学库
#include <cmath>
//sleep
#include "unistd.h"
#include <vector>
#include <algorithm> 

using namespace std;

namespace motor{

    typedef struct 
    {
        int x;
        int y;
    }Odom;

    class CMotorInitialized
    {
        public:        
            CMotorInitialized();
            CMotorInitialized(int id);
            ~CMotorInitialized();

            void setMotorMode(int mod);

            void setVel(double L_vel);
            void setTorque(double torque);
            
            int getVel(int id);
            void positionComputers(double v_l_rpm, double v_r_rpm);
        private:
            void sendMsgInit();
            uint8_t crc8_MAXIM(uint8_t *data, uint8_t len);
            serial::Serial* MotorSerial;
            std::mutex rv; // 接收数据锁
    };


    class CMotorInitializedSimple
    {
        public:        
            CMotorInitializedSimple();
            CMotorInitializedSimple(int id);
            ~CMotorInitializedSimple();

            void setMotorMode(int mod,int id);

            void setVel(double L_vel,double R_vel);
            void setTorque(double torque);
            
            int getVel(int id);
            void positionComputers(double v_l_rpm, double v_r_rpm);
        private:
            void sendMsgInit();
            uint8_t crc8_MAXIM(uint8_t *data, uint8_t len);
            serial::Serial* MotorSerial;
            std::mutex rv; // 接收数据锁
    };

    class CUwbPositionModel
    {
        public:
            CUwbPositionModel();
            ~CUwbPositionModel();
            bool getPosition();
            bool getPositionV5();
            std::vector<int> x_buffer_real;
            std::vector<int> y_buffer_real;
            Odom* odom;
        private:
            void uwbInitialize();
            void csvWriteInitial();
            serial::Serial* UwbSerial;
            std::vector<int> x_buffer;
            std::vector<int> y_buffer;
            std::ofstream uwb_csv_write;//写csv
    };
};




