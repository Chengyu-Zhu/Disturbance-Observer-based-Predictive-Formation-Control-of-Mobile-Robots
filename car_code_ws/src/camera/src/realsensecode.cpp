#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "camera/message.h"

void usage(const char *argv[], int error)
{
    std::cout << "SYNOPSIS" << std::endl
    << " " << argv[0] << " [--fps <6|15|30|60>]"
    << " [--width <image width>]"
    << " [--height <image height>]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
    std::cout << "DESCRIPTION" << std::endl
    << " --fps <6|15|30|60>" << std::endl
    << " Frames per second." << std::endl
    << " Default: 30." << std::endl
    << std::endl
    << " --width <image width>" << std::endl
    << " Default: 640." << std::endl
    << std::endl
    << " --height <image height>" << std::endl
    << " Default: 480." << std::endl
    << std::endl
    << " --seqname <sequence name>" << std::endl
    << " Name of the sequence of image to create (ie: /tmp/image%04d.jpg)." << std::endl
    << " Default: empty." << std::endl
    << std::endl
    << " --record <mode>" << std::endl
    << " Allowed values for mode are:" << std::endl
    << " 0: record all the captures images (continuous mode)," << std::endl
    << " 1: record only images selected by a user click (single shot mode)." << std::endl
    << " Default mode: 0" << std::endl
    << std::endl
    << " --no-display" << std::endl
    << " Disable displaying captured images." << std::endl
    << " When used and sequence name specified, record mode is internally set to 1 (continuous mode)."
    << std::endl
    << std::endl
    << " --help, -h" << std::endl
    << " Print this helper message." << std::endl
    << std::endl;
    std::cout << "USAGE" << std::endl
    << " Example to visualize images:" << std::endl
    << " " << argv[0] << std::endl
    << std::endl
    << " Examples to record a sequence of successive images in 640x480 resolution:" << std::endl
    << " " << argv[0] << " --seqname I%04d.png" << std::endl
    << " " << argv[0] << " --seqname folder/I%04d.png --record 0" << std::endl
    << std::endl
    << " Examples to record single shot 640x480 images:\n"
    << " " << argv[0] << " --seqname I%04d.png --record 1\n"
    << " " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl
    << " Examples to record single shot 1280x720 images:\n"
    << " " << argv[0] << " --seqname I%04d.png --record 1 --width 1280 --height 720" << std::endl
    << std::endl;
    if (error) {
        std::cout << "Error" << std::endl
        << " "
        << "Unsupported parameter " << argv[error] << std::endl;
    }
}
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
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    try {                           //satrt
    std::string opt_seqname;
    int opt_record_mode = 0;
    int opt_barcode = 0;
    int opt_fps = 40;
    bool opt_display = true;
    unsigned int opt_width = 640;
    unsigned int opt_height = 480;
    // for (int i = 1; i < argc; i++) {
    //     if (std::string(argv[i]) == "--fps") {
    //         opt_fps = std::atoi(argv[i + 1]);
    //         i++;
    //     } else if (std::string(argv[i]) == "--seqname") {
    //         opt_seqname = std::string(argv[i + 1]);
    //         i++;
    //     } else if (std::string(argv[i]) == "--width") {
    //         opt_width = std::atoi(argv[i + 1]);
    //         i++;
    //     } else if (std::string(argv[i]) == "--height") {
    //         opt_height = std::atoi(argv[i + 1]);
    //         i++;
    //     } else if (std::string(argv[i]) == "--record") {
    //         opt_record_mode = std::atoi(argv[i + 1]);
    //         i++;
    //     } else if (std::string(argv[i]) == "--no-display") {
    //         opt_display = false;
    //     } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
    //         usage(argv, 0);
    //         return EXIT_SUCCESS;
    //     } else {
    //         usage(argv, i);
    //         return EXIT_FAILURE;
    //     }
    // }
    if ((!opt_display) && (!opt_seqname.empty())) {
        opt_record_mode = 0;
    }
    if (opt_fps != 6 && opt_fps != 15 && opt_fps != 30 && opt_fps != 60) {
        opt_fps = 30; // Default
    }
    std::cout << "Resolution : " << opt_width << " x " << opt_height << std::endl;
    std::cout << "Recording : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Framerate : " << opt_fps << std::endl;
    std::cout << "Display : " << (opt_display ? "enabled" : "disabled") << std::endl;
    std::string text_record_mode =
    std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));
    if (!opt_seqname.empty()) {
        std::cout << text_record_mode << std::endl;
        std::cout << "Record name: " << opt_seqname << std::endl;
    }
    vpRealSense2 g;
    vpImage<unsigned char> I;

#ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK : Realsense 2" << std::endl;
    rs2::config config;
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_DEPTH, opt_width, opt_height, RS2_FORMAT_Z16, opt_fps);
    config.enable_stream(RS2_STREAM_COLOR, opt_width, opt_height, RS2_FORMAT_RGBA8, opt_fps);
    g.open(config);
    vpImage<vpRGBa> Ic(g.getIntrinsics(RS2_STREAM_COLOR).height, g.getIntrinsics(RS2_STREAM_COLOR).width);
    vpImage<uint16_t> Id_raw(g.getIntrinsics(RS2_STREAM_DEPTH).height, g.getIntrinsics(RS2_STREAM_DEPTH).width);
    vpImage<vpRGBa> Id(g.getIntrinsics(RS2_STREAM_DEPTH).height, g.getIntrinsics(RS2_STREAM_DEPTH).width);
    rs2::align align_to(RS2_STREAM_COLOR);
#else
    std::cout << "SDK : Realsense 1" << std::endl;
    vpRealSense g;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(opt_width, opt_height, rs::format::rgba8, 60));
    g.open();
#endif
    g.acquire(I);
    g.acquire((unsigned char *) Ic.bitmap, (unsigned char *) Id_raw.bitmap, NULL, NULL, &align_to);
    //g.acquire(NULL,(unsigned char *)Id_raw.bitmap,NULL,NULL,NULL);
    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;
    vpDisplay *d = NULL;
    if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    std::cout << "No image viewer is available..." << std::endl;
    opt_display = false;
#endif
    }
    if (opt_display) {
#ifdef VISP_HAVE_X11
    d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
    d = new vpDisplayGDI(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
    d = new vpDisplayOpenCV(I);
#endif
    }
    vpDetectorBase *detector = NULL;
#if (defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX))
    if (opt_barcode == 0)
    detector = new vpDetectorQRCode;
else
    detector = new vpDetectorDataMatrixCode;
#elif defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
    (void)opt_barcode;
#elif defined(VISP_HAVE_DMTX)
    detector = new vpDetectorDataMatrixCode;
    (void)opt_barcode;
#endif
    vpImageQueue<vpRGBa> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<vpRGBa> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_storage_worker);
    bool quit = false;
    while (!quit) {
        double t = vpTime::measureTimeMs();
        g.acquire(I);
        g.acquire((unsigned char *) Ic.bitmap, (unsigned char *) Id_raw.bitmap, NULL, NULL, &align_to);
        //g.acquire(NULL,(unsigned char *)Id_raw.bitmap,NULL,NULL,NULL);
        vpDisplay::display(I);
        vpDisplay::display(Ic);
        float depth=0;
        float w=0;
        float h=0;
        int count=0;
        bool status = detector->detect(I);
        std::ostringstream legend;
        legend << detector->getNbObjects() << " bar code detected";
        vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);
        if (status) {
            for (size_t i = 0; i < detector->getNbObjects(); i++) {
                std::vector<vpImagePoint> p = detector->getPolygon(i);
                vpRect bbox = detector->getBBox(i);
                
                vpDisplay::displayRectangle(I, bbox, vpColor::green);
                //vpDisplay::displayText(I, (int)bbox.getTop() - 20, (int)bbox.getLeft(),
                //"Message: \"" + detector->getMessage(i) + "\"", vpColor::red);

                    for (size_t j = 0; j < p.size(); j++) {
                        vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
                        std::ostringstream number;
                        number << j;
                        vpDisplay::displayText(I, p[j] + vpImagePoint(20, 10), number.str(), vpColor::yellow);
                        std::ostringstream number_1;
                        number_1 << Id_raw(p[j]);
                        if(Id_raw(p[j])!=0){
                            depth = depth + Id_raw(p[j]);
                            count = count + 1;
                        }
                        w = w+p[j].get_i();
                        h = h+p[j].get_j();
                        vpDisplay::displayText(I, p[j] + vpImagePoint(-20, -10), number_1.str(), vpColor::blue);
                        std::ostringstream number_2;
                        number_2 << p[j]<<p[j];
                        vpDisplay::displayText(I, p[j] + vpImagePoint(40, -30), number_2.str(), vpColor::red);
                    }
                    p.clear();
                std::ostringstream number_3;
                number_3 << "center depth is: " << depth/count;
                vpDisplay::displayText(I, I.getHeight() - 40, 10, number_3.str(), vpColor::red);
                depth = 0;

                std::ostringstream number_5;
                number_5 << "center point is: " << w/4 <<" "<<h/4;
                vpDisplay::displayText(I, I.getHeight() - 60, 10, number_5.str(), vpColor::red);

                std::stringstream ss;
                ss <<(h/4 -320)/616 << ","<<(w/4 - 240)/616;
                msg.data = ss.str();
                image_data_pub.publish(msg);
                ROS_INFO("%s",msg.data.c_str());

            }
        }
        std::stringstream ss;
        ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
        vpDisplay::displayText(I, I.getHeight() - 20, 10, ss.str(), vpColor::red);
        vpDisplay::flush(I);
        vpDisplay::flush(Ic);
    }
    image_queue.cancel();
    image_storage_thread.join();
        if (d) {
        delete d;
        }
    }                       //end
catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
}
#else
    (void)argc;
    (void)argv;
#if !(defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2))
    std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
