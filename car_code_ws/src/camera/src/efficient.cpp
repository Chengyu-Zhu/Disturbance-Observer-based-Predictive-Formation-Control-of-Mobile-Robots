#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/blob/vpDot2.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
using namespace std;
using namespace cv;

int choose_index = 0;
void call_back(const geometry_msgs::Twist::ConstPtr &p)
{
    ROS_INFO("hhh");
    if(p->linear.x==0.5){
        ROS_INFO("hhh--");
        choose_index = 33;
    }
};

int main(int argc, char *argv[])
{

    ros::init(argc,argv,"velocity_publisher");
	//创建句柄-管理节点资源
	ros::NodeHandle n;
	//创建一个Publisher，发布名为/image 的tpoic，消息类型是std_msgs::String，队列长度为100
    ros::Publisher image_data_pub = n.advertise<std_msgs::String>("/image",1);
    ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/motor_speedss", 1, call_back);
	//循环频率
    std_msgs::String msg;
	ros::Rate loop_rate(10);

    //image parameters
    int width = 640, height = 480, fps = 30;
    float d435i_depth=0;
    float d435i_w=0;
    float d435i_h=0;
    int d435i_count=0;
    int count = 0;
    long icount = 0;
    //camera initialization
    vpRealSense2 rs;
    rs2::config config;
    rs2::frame frame;
    rs2::align align_to(RS2_STREAM_COLOR);
    //config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, fps);
    config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    rs.open(config);
    //image initialization
    vpImage<vpRGBa> depth_color(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
    vpImage<uint16_t> depth_raw(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
    vpImage<unsigned char> infrared1(static_cast<unsigned int>(height), static_cast<unsigned int>(width));
    //vpImage<unsigned char> infrared1;
    //show image choice
    vpDisplayX *d435i = NULL;
    //d435i.init(infrared1, 0, 0, "Infrared");
    d435i = new vpDisplayX(infrared1);
    std::vector<vpColVector> pointcloud_colvector;
    
    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);

    vpImagePoint germ;
    bool init_done = false;
    std::cout << "Click!!!" << std::endl;
    choose_index = 0;
    while (true) {
        ros::spinOnce();
        try {
            rs.acquire(infrared1,NULL);
            rs.acquire(reinterpret_cast<unsigned char *>(depth_color.bitmap),reinterpret_cast<unsigned char *>(depth_raw.bitmap),NULL,NULL,&align_to);
            vpDisplay::display(infrared1);
            std::ostringstream legend;
            vpDisplay::displayText(infrared1, 10, 10, "Click to quit.", vpColor::red);
            d435i_count=0;
            d435i_depth=0;
            d435i_h=0;
            d435i_w=0;
            if (!init_done) {
                vpDisplay::displayText(infrared1, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
                if (vpDisplay::getClick(infrared1, germ, false)) {
                    blob.initTracking(infrared1, germ);
                    init_done = true;
                }
            }
            else {
                blob.track(infrared1);
            }
            vpDisplay::flush(infrared1);
            std::cout<<blob.getCog()<<std::endl;
            blob.getCog();
            std::stringstream ss;
            icount++;
            if(choose_index == 0)
            {
                ss <<(blob.getCog().get_u() -320)/616 << ","<<(blob.getCog().get_v() - 240)/616<<","<<icount<<","<<1;
            }else if(choose_index == 33){
                ss <<(blob.getCog().get_u() -320)/616 << ","<<(blob.getCog().get_v() - 240)/616<<","<<icount<<","<<2;
            }
            
            msg.data = ss.str();
            image_data_pub.publish(msg);
            ROS_INFO("%s",msg.data.c_str());
            count = 0;            
        }
        catch (...) {
            init_done = false;
        }

        vpDisplay::flush(infrared1);
        //click stop button
        if (vpDisplay::getClick(infrared1, false)) {
        break;
        }
    }
    //close
    rs.close();
    d435i->close(infrared1);
    return EXIT_SUCCESS;
}

    






