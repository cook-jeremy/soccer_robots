#include "ros/ros.h"
#include "std_msgs/String.h"
#include <robot_comm/Motor.h>
#include <sstream>
#include <signal.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_main");
    ros::NodeHandle n;
    
    ros::Publisher command_pub = n.advertise<robot_comm::Motor>("motor", 1000);
    
    ros::Rate loop_rate(1);
    int count = 255;
    while (ros::ok())
    {
        robot_comm::Motor msg;
        msg.name = "Robot 1";
        msg.left_power = count;
        msg.right_power = 255;
        
        command_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count--;
    }
    return 0;
}