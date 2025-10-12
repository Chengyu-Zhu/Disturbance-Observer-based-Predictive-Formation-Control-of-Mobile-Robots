#include <ros/ros.h>
#include <std_msgs/String.h>
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

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{

  ros::init(argc,argv,"velocity_publisher");
	//创建句柄-管理节点资源
	ros::NodeHandle n;
	//创建一个Publisher，发布名为/image 的tpoic，消息类型是std_msgs::String，队列长度为100
  ros::Publisher image_data_pub = n.advertise<std_msgs::String>("/image",1);
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
  //
  vpDetectorBase *detector = NULL;
  detector = new vpDetectorQRCode;
  while (true) {
    //rs.acquire(infrared1);
    rs.acquire(infrared1,NULL);
    rs.acquire(reinterpret_cast<unsigned char *>(depth_color.bitmap),reinterpret_cast<unsigned char *>(depth_raw.bitmap),NULL,NULL,&align_to);
    vpDisplay::display(infrared1);
    bool status = detector->detect(infrared1);
    std::cout<<status<<std::endl;
    std::ostringstream legend;
    legend << detector->getNbObjects() << " bar code detected";
    vpDisplay::displayText(infrared1, 10, 10, "Click to quit.", vpColor::red);
    d435i_count=0;
    d435i_depth=0;
    d435i_h=0;
    d435i_w=0;
    if (status) {
      for (size_t i = 0; i < detector->getNbObjects(); i++) {
        std::vector<vpImagePoint> p = detector->getPolygon(i);
        vpRect bbox = detector->getBBox(i);
        vpDisplay::displayRectangle(infrared1, bbox, vpColor::green);
        // vpDisplay::displayText(infrared1, (int)bbox.getTop() - 20, (int)bbox.getLeft(),
        //                         "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
        for (size_t j = 0; j < p.size(); j++) {
          vpDisplay::displayCross(infrared1, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(infrared1, p[j] + vpImagePoint(10, 0), number.str(), vpColor::blue);
          std::ostringstream depth_row_number;
          // depth_row_number << depth_raw(p[j]);
          // if(depth_raw(p[j]) != 0){
            // d435i_depth = d435i_depth + depth_raw(p[j]);
            // d435i_count = d435i_count + 1;
          // }
          d435i_w = d435i_w+p[j].get_i();
          d435i_h = d435i_h+p[j].get_j();
          // vpDisplay::displayText(infrared1, p[j] + vpImagePoint(-20, -10), depth_row_number.str(), vpColor::blue);
          std::ostringstream number_2;
          number_2 << p[j]<<p[j];
          vpDisplay::displayText(infrared1, p[j] + vpImagePoint(40, -30), number_2.str(), vpColor::red);
        }
        p.clear();
      }
      std::ostringstream number_3;
      number_3 << "center depth is: " << d435i_depth/d435i_count;
      vpDisplay::displayText(infrared1, infrared1.getHeight() - 40, 10, number_3.str(), vpColor::red);
      d435i_depth = 0;

      std::ostringstream number_5;
      number_5 << "center point is: " << d435i_w/4 <<" "<<d435i_h/4;
      vpDisplay::displayText(infrared1, infrared1.getHeight() - 60, 10, number_5.str(), vpColor::red);

      std::stringstream ss;
      ss <<(d435i_h/4 -320)/616 << ","<<(d435i_w/4 - 240)/616;
      msg.data = ss.str();
      image_data_pub.publish(msg);
      ROS_INFO("%s",msg.data.c_str());
      count = 0;
    }
    else if (count == 5){
      std::stringstream ss;
      ss <<1000 << ","<<1000;
      msg.data = ss.str();
      image_data_pub.publish(msg);
      ROS_INFO("%s",msg.data.c_str());
      count = 0;
    }
    else{
      count++;
    }
    vpDisplay::flush(infrared1);
    //click stop button
    if (vpDisplay::getClick(infrared1, false)) {
      break;
    }
  }
  //close
  delete detector;
  rs.close();
  d435i->close(infrared1);
  return EXIT_SUCCESS;
}

    






