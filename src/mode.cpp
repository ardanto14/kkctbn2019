#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Mode.h>
#include <mavros_msgs/RCIn.h>
#include <sstream>
ros::Publisher mode_publisher;


void pwmModeCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    ROS_WARN("%d", sizeof(msg));    
    int pwm = msg->channels[7];
    
    /**  
    if (pwm < 1400) {
        kkctbn2019::Mode mode;
        mode.value = kkctbn2019::Mode::HOLD;
        mode_publisher.publish(mode);
    } else if (pwm > 1600) {
        kkctbn2019::Mode mode;
        mode.value = kkctbn2019::Mode::MANUAL;
        mode_publisher.publish(mode);
    } else {
        kkctbn2019::Mode mode;
        mode.value = kkctbn2019::Mode::AUTO;
        mode_publisher.publish(mode);
    }
    */
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mode");
    ros::NodeHandle nh;

    ros::Subscriber pwm_mode_subscriber = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 8, pwmModeCallback);

    mode_publisher = nh.advertise<kkctbn2019::Mode>("/makarax/mode", 8);

    ROS_WARN("mode is active");

    ros::spin();
    
    return 0;
}



