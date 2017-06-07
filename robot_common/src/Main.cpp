#include <ros/ros.h>
#include <std_msgs/String.h>
#include "ColorLocation.h"


using namespace std;

class Main {
public:
    Main(ros::NodeHandle);
};

Main::Main(ros::NodeHandle n) {

    while (ros::ok())
    {

        ros::spinOnce();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    Main main(n);
    ros::spin();
    return 0;
}