#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Mode.h>
#include <mavros_msgs/RCIn.h>

ros::Publisher mode_publisher;


void pwmModeCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    int pwm = msg->channels[7];
    ROS_INFO("Kesini");
    if (pwm < 1400) {
        kkctbn2019::Mode mode;
        mode.value = 1;
        mode_publisher.publish(mode);
    } else if (pwm > 1600) {
        kkctbn2019::Mode mode;
        mode.value = 3;
        mode_publisher.publish(mode);
    } else {
        kkctbn2019::Mode mode;
        mode.value = 2;
        mode_publisher.publish(mode);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mode");
    ros::NodeHandle nh;

    ros::Subscriber pwm_mode_subscriber = nh.subscribe("/mavros/rc/in", 8, pwmModeCallback);

    mode_publisher = nh.advertise<kkctbn2019::Mode>("/makarax/mode", 8);

    ROS_WARN("mode is active");

    ros::spin();
    
    return 0;
}



