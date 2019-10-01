#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/AutoControl.h>
#include <mavros_msgs/RCIn.h>

ros::Publisher auto_control_publisher;


void pwmModeCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    int pwm = msg->channels[6];
    kkctbn2019::AutoControl control;
    if (pwm < 1400) {
        control.state = kkctbn2019::AutoControl::AVOID_RED;
    } else if (pwm > 1600) {
        control.state = kkctbn2019::AutoControl::AVOID_GREEN;
    } else {
        control.state = kkctbn2019::AutoControl::AVOID_RED_AND_GREEN;
    }
    mode_publisher.publish(mode);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mode");
    ros::NodeHandle nh;

    ros::Subscriber pwm_mode_subscriber = nh.subscribe("/mavros/rc/in", 8, pwmModeCallback);

    auto_control_publisher = nh.advertise<kkctbn2019::AutoControl>("/makarax/auto_control", 8);

    ROS_WARN("mode is active");

    ros::spin();
    
    return 0;
}



