#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Command.h>


void commandCallback(const kkctbn2019::Command::ConstPtr& msg) {
    if (msg->maju) {
        ROS_INFO("maju");
    } else if (msg->mundur) {
        ROS_INFO("mundur");
    } else if (msg->kiri) {
        ROS_INFO("kiri");
    } else if (msg->kanan) {
        ROS_INFO("kanan");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber command_subscriber = nh.subscribe("/makarax/command", 1000, commandCallback);

    ROS_WARN("controller is active");

    ros::spin();
    
    return 0;
}



