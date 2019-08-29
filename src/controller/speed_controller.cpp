#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <kkctbn2019/Speed.h>

using namespace std;

int tombol_tambah, tombol_kurang;
int jumlah = 50;

ros::Publisher speed_publisher;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	tombol_tambah = joy->buttons[0];
    tombol_kurang = joy->buttons[1];
    
    if(tombol_tambah > 0 && jumlah < 200){
        jumlah += 50;
        kkctbn2019::Speed speed;
        speed.spd = jumlah;
        speed_publisher.publish(speed);
        cout << speed.spd << endl;
    }
    else if (tombol_kurang>0 && jumlah > 0){
        jumlah -= 50;
        kkctbn2019::Speed speed;
        speed.spd = jumlah;
        speed_publisher.publish(speed);
        cout << speed.spd << endl;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle n;

    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    speed_publisher = n.advertise<kkctbn2019::Speed>("chatter", 8);

    ros::spin();
    return 0;
}