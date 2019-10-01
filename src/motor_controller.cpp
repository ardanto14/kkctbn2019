#include <iostream>
#include <string>
#include <ros/ros.h>
#include <kkctbn2019/Command.h>
#include <kkctbn2019/Mode.h>
#include <kkctbn2019/AutoControl.h>
#include <kkctbn2019/ObjectCount.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <stdlib.h>
#include <sensor_msgs/Joy.h>

ros::Publisher override_publisher;
ros::Publisher throttle_pwm_publisher;
ros::Publisher state_publisher;
kkctbn2019::Mode mode;
kkctbn2019::AutoControl autoControl;
float control_effort;
int currentThrottlePwm = 1700;
int green = 0;

void greenCallback(const std_msgs::UInt16::ConstPtr& msg) {
    green = msg->data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    if (msg->buttons[0] == 1) {
        currentThrottlePwm += 50;
    } else if (msg->buttons[2] == 1) {
        currentThrottlePwm -= 50;
    }

    if (currentThrottlePwm > 1900) {
        currentThrottlePwm = 1900;
    }

    if (currentThrottlePwm < 1600) {
        currentThrottlePwm = 1600;
    }
}

void controlEffortCallback(const std_msgs::Float64::ConstPtr& msg) {
    control_effort = msg->data;
}

void modeCallback(const kkctbn2019::Mode::ConstPtr& msg) {
    mode = *msg;
    if (mode.value == kkctbn2019::Mode::MANUAL) {
        mavros_msgs::OverrideRCIn rcin;
        for (int i = 0; i < 8; i ++) rcin.channels[i] = 0;
        override_publisher.publish(rcin);
    }
}

void autoControlCallback(const kkctbn2019::AutoControl::ConstPtr& msg) {
    autoControl = *msg;
}

void objectCountCallback(const kkctbn2019::ObjectCount::ConstPtr& msg) {
    // ROS_INFO("Current Throttle is %d", currentThrottlePwm);
    std_msgs::UInt16 throttle_pwm;
    throttle_pwm.data = currentThrottlePwm;
    throttle_pwm_publisher.publish(throttle_pwm);
<<<<<<< HEAD
    if (mode.value == kkctbn2019::Mode::AUTO) {
        if (autoControl.state == kkctbn2019::AutoControl::AVOID_RED_AND_GREEN) {
            if (msg->red > 0){
                mavros_msgs::OverrideRCIn rcin;
                for (int i = 0; i < 8; i ++) rcin.channels[i] = 0;
                rcin.channels[2] = currentThrottlePwm;
                rcin.channels[0] = 1500 + control_effort;
                if (rcin.channels[0] > 2200) {
                    rcin.channels[0] = 2200;
                }
                else if (rcin.channels[0] < 800) {
                    rcin.channels[0] = 800;
                }
                override_publisher.publish(rcin);
            } else if (msg->green > 0) {
                mavros_msgs::OverrideRCIn rcin;
                for (int i = 0; i < 8; i ++) rcin.channels[i] = 0;
                rcin.channels[2] = currentThrottlePwm;
                rcin.channels[0] = 1650;
                override_publisher.publish(rcin);
            } else {
                mavros_msgs::OverrideRCIn rcin;
                rcin.channels[2] = currentThrottlePwm;
                rcin.channels[0] = 1600;
                override_publisher.publish(rcin);
            }
        } else {
            if (msg->red > 0) {
                mavros_msgs::OverrideRCIn rcin;
                for (int i = 0; i < 8; i ++) rcin.channels[i] = 0;
                rcin.channels[2] = currentThrottlePwm;
                rcin.channels[0] = 1500 + control_effort;
                if (rcin.channels[0] > 2200) {
                    rcin.channels[0] = 2200;
                }
                else if (rcin.channels[0] < 800) {
                    rcin.channels[0] = 800;
                }
                override_publisher.publish(rcin);
            } else {
                mavros_msgs::OverrideRCIn rcin;
                rcin.channels[2] = currentThrottlePwm;
                rcin.channels[0] = 1600;
                override_publisher.publish(rcin);
        }
=======
    if (mode.value == kkctbn2019::Mode::MANUAL) {
        // ROS_INFO("MANUAL");
    } 
    else if (mode.value == kkctbn2019::Mode::AUTO) {
        // ROS_INFO("AUTO");
	if (green > 0 && zzz->data > 0) {
	    mavros_msgs::OverrideRCIn rcin;
	    rcin.channels[2] = currentThrottlePwm;
		rcin.channels[0] = 1500;
		override_publisher.publish(rcin);
	} else if (green > 0 && zzz->data == 0){
		ROS_INFO("green");
            mavros_msgs::OverrideRCIn rcin;
            rcin.channels[2] = currentThrottlePwm;
            rcin.channels[0] = 1675;
            override_publisher.publish(rcin);
        } else if (zzz->data > 0) {
		ROS_INFO("red");
            mavros_msgs::OverrideRCIn rcin;
            for (int i = 0; i < 8; i ++) rcin.channels[i] = 0;
            rcin.channels[2] = currentThrottlePwm;
            rcin.channels[0] = 1500 + control_effort;
            if (rcin.channels[0] > 2200) {
                rcin.channels[0] = 2200;
            }
            else if (rcin.channels[0] < 800) {
                rcin.channels[0] = 800;
            }
            override_publisher.publish(rcin);
        } else {
	    mavros_msgs::OverrideRCIn rcin;
	    rcin.channels[2] = currentThrottlePwm;
            rcin.channels[0] = 1500;
            override_publisher.publish(rcin);
	}
    } else {
        // ROS_INFO("HOLD");
>>>>>>> 42722677865d2dc98425a44db44dbea8c6178fc0
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;

    override_publisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 8);

    throttle_pwm_publisher = nh.advertise<std_msgs::UInt16>("/makarax/pwm/throttle", 8);

    ros::Subscriber mode_subscriber = nh.subscribe("/makarax/mode", 8, modeCallback);

    ros::Subscriber control_effort_subscriber = nh.subscribe("control_effort", 8, controlEffortCallback);
    
<<<<<<< HEAD
    ros::Subscriber red_count_subscriber = nh.subscribe("/makarax/object/count",8, objectCountCallback);

=======
    ros::Subscriber control_effort1_subscriber = nh.subscribe("/makarax/object/count/red",8, modeCallback1);
    ros::Subscriber green_subscriver = nh.subscribe("/makarax/object/count/green", 8, greenCallback);
>>>>>>> 42722677865d2dc98425a44db44dbea8c6178fc0
    ros::Subscriber joy_subscriber = nh.subscribe("joy", 8, joyCallback);

    ros::Subscriber auto_control_subscriber = nh.subscribe("/makarax/auto_control", 8, autoControlCallback);

    ROS_WARN("controller is active");

    ros::spin();
    
    return 0;
}



