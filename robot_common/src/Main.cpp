#include <ros/ros.h>
#include <std_msgs/String.h>
#include <RobotNames.h>
#include "ImageHandler.h"
#include "RobotNames.h"
#include "Robot.h"

using namespace std;

class Main {
public:
    Main(ros::NodeHandle);
    std::vector<Robot> setupRobots();
    void findRobotPositions(std::vector<Robot>, std::vector< std::vector<ColorLocation> >);
};

Main::Main(ros::NodeHandle n) {
    image_transport::ImageTransport it(n);
    ImageHandler ih(n, it);

    std::vector<Robot> robots = setupRobots();

    cout << "Starting Main" << endl;
    while (ros::ok())
    {
        // Get all colors from the image
        std::vector< std::vector<ColorLocation> > colors = ih.getAllColors();

        findRobotPositions(robots, colors);

        ros::spinOnce();
    }
}

void Main::findRobotPositions(std::vector<Robot> robots, std::vector< std::vector<ColorLocation> > colors) {
    for(int i = 0; i < robots.size(); i++) {

        // Find vertical pair left
        // Find vertical pair right
        // Find diagonal pair top left bottom right
        // Find diagonal pair top right bottom left
        // Do I need to find any other pairs?

    }
}



std::vector<Robot> Main::setupRobots() {
    std::vector<Robot> robots;
    Robot robot1(ROBOT_1_NAME, ROBOT_1_IP);
    robot1.setColors(ROBOT_1_TOP_LEFT, ROBOT_1_TOP_RIGHT, ROBOT_1_BOTTOM_LEFT, ROBOT_1_BOTTOM_RIGHT);

    robots.push_back(robot1);
    return robots;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    Main main(n);
    ros::spin();
    return 0;
}