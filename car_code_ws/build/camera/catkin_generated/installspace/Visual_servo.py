#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist

print('cv2.__version__:',cv2.__version__)

#load model
path = '/home/zcy-robot/exp1_ws/devel/lib/camera/'
detect_obj = cv2.wechat_qrcode_WeChatQRCode(path+'detect.prototxt',path+'detect.caffemodel',path+'sr.prototxt',path+'sr.caffemodel')

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
	if s.get_info(rs.camera_info.name) == 'RGB Camera':
		found_rgb = True
		break
if not found_rgb:
	print("The demo requires Depth camera with Color sensor")
	exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

if device_product_line == 'L500':
	config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
else:
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

# Start streaming
pipeline.start(config)

def main():

	# ROS节点初始化
    rospy.init_node('control', anonymous=True)

	# 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        #while True:
        # Wait for a coherent pair of frames: depth and color

        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        #if not depth_frame or not color_frame:
        if not color_frame:
            continue

        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        #depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        #     images = np.hstack((resized_color_image, depth_colormap))
        # else:
            # images = np.hstack((color_image, depth_colormap))
            #images = np.hstack((color_image, depth_colormap))

        res,points = detect_obj.detectAndDecode(color_image)
        print('res:',res)
        print('points:',points)
        for pos in points:
            color=(0,0,255)
            thick=3
            for p in [(0,1),(1,2),(2,3),(3,0)]:
                start = int(pos[p[0]][0]),int(pos[p[0]][1])
                end = int(pos[p[1]][0]),int(pos[p[1]][1])
                cv2.line(color_image,start,end,color,thick)

		# 初始化geometry_msgs::Twist类型的消息
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2

		# 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z)
        
		# Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
		# Stop streaming
    	pipeline.stop()

