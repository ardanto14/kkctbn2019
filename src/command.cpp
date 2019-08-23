#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Command.h>
#include <std_msgs/UInt16.h>

ros::Publisher command_publisher;

void objectCountCallback(const std_msgs::UInt16::ConstPtr& msg) {
    if (msg->data > 0) {
        kkctbn2019::Command command;
        command.value = kkctbn2019::Command::KANAN;
        command_publisher.publish(command);
    } else {
        kkctbn2019::Command command;
        command.value = kkctbn2019::Command::KIRI;
        command_publisher.publish(command);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command");
    ros::NodeHandle nh;

    command_publisher = nh.advertise<kkctbn2019::Command>("/makarax/command", 8);
    ros::Subscriber object_count_subscriber = nh.subscribe("/makarax/object/count", 8, objectCountCallback);

    ROS_WARN("AI is active");
    
    ros::spin();

    return 0;
}



