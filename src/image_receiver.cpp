#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;

Mat image;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_receiver");
	ros::NodeHandle nh;
    ros::Publisher image_publisher = nh.advertise<sensor_msgs::Image>("/makarax/image", 1);
	
	ROS_WARN("image_receiver is active");
	
	VideoCapture cap(0); 
	if(!cap.isOpened()){
		ROS_ERROR ("Error opening camera.");	  
		return 1;
	}

	while (ros::ok()) {
		cap.read(image);		
		if(!image.empty()){	
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			image_publisher.publish(msg);
		}
		ros::spinOnce();
	}
}