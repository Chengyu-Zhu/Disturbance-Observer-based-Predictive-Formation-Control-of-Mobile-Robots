#include <thread>
#include <iostream>

#include <ros/ros.h>

#include "can/motor_init.h"
#include "gust/robot.h"
#include "gust/cmd.h"

motor_init mi;

bool run_flag;

/**
 * @brief 获取电机转速，针对负转速进行相应的转换
 * 
 * @param data 串口转速数据
 * \return RPM 实际的转速
 */
int data_change(int data)
{
    if (data > 32768)
    {
        data = data - 65536;
        return data;
    }
    else
    {
        return data;
    }
}



int Can_RPM_L;
int Can_RPM_R;
double Vel_L;
double Vel_R;

double vel_rectify = 0.812;//速度校正值

/**
 * @brief 速度转换
 * 
 * @param id 通道编号
 * @param read 读取的数据
 */
void read_vel(int id, VCI_CAN_OBJ read)
{             
    int Read_RPM;//读取的转速数据
    Read_RPM = read.Data[4] | read.Data[5] << 8;
    if (id == 0)
    {
        Can_RPM_R = data_change(Read_RPM);
        Vel_R = Can_RPM_R * M_PI * 0.168 * 0.1 / 60 * vel_rectify;
        //std::cout << vel_right << std::endl;
    }
    if (id == 1)
    {
        Can_RPM_L = -data_change(Read_RPM);
        Vel_L = Can_RPM_L * M_PI * 0.168 * 0.1 / 60 * vel_rectify;
        //std::cout << Can_RPM_L << std::endl;
    }
}


std::mutex rv;
/**
 * @brief 串口数据接收
 * 
 */
void receive_func() 
{

    int reclen = 0;
    VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳
    int j;
    
    int ind = 0;//CAN设备通道
    int Read_RPM;//读取的转速数据

    run_flag = true;
    while(run_flag)
    {

        mi.Get_Velocity();

        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) //调用接收函数，如果有数据，进行数据处理显示。
        {

            for(j = 0; j < reclen; j++)
            {
                //转速读取
                if (rec[j].Data[0] == 0x43 && rec[j].Data[1] == 0x06 && rec[j].Data[2] == 0x30 && rec[j].Data[3] == 0x00)
                {
                    std::lock_guard<std::mutex> lockGuard(rv);
                    read_vel(ind,rec[j]);
                }
            }
        }
        ind = !ind; //变换通道号，以便下次读取另一通道，交替读取。

        usleep(100000);

    }

    
    std::cout << "run 0 thread exit" << std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Gust");
    //robot
    Gust Robot_Control;        //Instantiate an object //实例化一个对象

    //CMD
    Cmd Cmd_connect;

    std::thread vt(receive_func);
    int b = 0;
    


    std::cout << "start!" << std::endl;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
        Robot_Control.begin_time = ros::Time::now();
        Robot_Control.vel_L = Vel_L;
        Robot_Control.vel_R = Vel_R;
        Robot_Control.Control();  

        
        Cmd_connect.cmd_move_control();
        mi.Set_Velocity(Cmd_connect.leftValue,Cmd_connect.rightValue);


        Robot_Control.last_time = Robot_Control.begin_time;
        ros::spinOnce();
        loop_rate.sleep();

    }

    std::cout << "close" << std::endl;


    run_flag = false; //关闭线程

    vt.join(); //等待线程结束

    return 0;
}