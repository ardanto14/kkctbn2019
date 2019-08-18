#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Command.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "command");
    ros::NodeHandle nh;

    ros::Publisher command_publisher = nh.advertise<kkctbn2019::Command>("/makarax/command", 1000);

    ROS_WARN("Command is active");

    while (ros::ok()) {
        kkctbn2019::Command command;
        command.kanan = false;
        command.kiri = false;
        command.maju = false;
        command.mundur = true;
        command_publisher.publish(command);
        ros::spinOnce();
    }

    return 0;
}



