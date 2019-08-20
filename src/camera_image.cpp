#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

using namespace std;
using namespace cv;

Mat image;

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_camera = it.advertise("/camera/image", 1, true);

    //camera 0
    VideoCapture cap(0);
    if(!cap.isOpened()){
        cout << "Ccamera ERROR" << endl;
        return 1;
    }
    while(nh.ok()){
        cap.read(image);
        if(!image.empty()){
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub_camera.publish(msg); //publish image dalam bentuk cv_bridge
        }
        ros::spinOnce();
    }
}