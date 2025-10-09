
#ifndef __ROBOT_H_
#define __ROBOT_H_

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>          
#include <stdbool.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

using namespace std;


extern sensor_msgs::Imu HFIA9; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __HFIA9_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}HFIA9_DATA;



//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class Gust
{
	public:
		Gust();  //Constructor //构造函数
		~Gust(); //Destructor //析构函数

		ros::Time begin_time, last_time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		void Control();   //Loop control code //循环控制代码
		float vel_L,vel_R;
		
	private:
		ros::NodeHandle br;          //Create a ROS node handle //创建ROS节点句柄
		
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

		ros::Publisher odom_publisher = br.advertise<nav_msgs::Odometry>("odom", 100);           //Create the odometer topic publisher //创建里程计话题发布者
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题

        //读取运动底盘速度、IMU、电源电压数据
        void Get_Sensor_Data();

        string robot_frame_id, odom_frame_id; //Define the related variables //定义相关变量

        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度




};
#endif
