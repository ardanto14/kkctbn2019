#include <iostream>
#include <ros/ros.h>
#include <kkctbn2019/Mode.h>
#include <mavros_msgs/RCIn.h>
#include <sstream>
ros::Publisher mode_publisher;


void pwmModeCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    ROS_WARN("%d", sizeof(msg));    
    int pwm = msg->channels[7];
<<<<<<< HEAD
    kkctbn2019::Mode mode;
=======
    
    /**  
>>>>>>> 42722677865d2dc98425a44db44dbea8c6178fc0
    if (pwm < 1400) {
        mode.value = kkctbn2019::Mode::HOLD;
    } else if (pwm > 1600) {
        mode.value = kkctbn2019::Mode::MANUAL;
    } else {
        mode.value = kkctbn2019::Mode::AUTO;
    }
<<<<<<< HEAD
    mode_publisher.publish(mode);
=======
    */
>>>>>>> 42722677865d2dc98425a44db44dbea8c6178fc0
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



