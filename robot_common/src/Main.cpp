#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Config.h"
#include "ColorPair.h"
#include "ImageHandler.h"
#include "Robot.h"

using namespace std;

class Main {
public:
    Main(ros::NodeHandle);
    std::vector<Robot> setupRobots();
    void findRobotPositions(std::vector<Robot>, std::vector< std::vector<ColorLocation> >);
    std::vector<ColorPair> findDiagonal(std::vector< std::vector<ColorLocation> >, std::string, std::string);
    bool inDiagonalRange(ColorLocation, ColorLocation);
    void findCorrectDiagonals(Robot, std::vector<ColorPair>, std::vector<ColorPair>);
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

        // Find diagonal pair top left bottom right
        std::vector<ColorPair> potential_pairs1 = findDiagonal(colors, robots.at(i).getTopLeft(), robots.at(i).getBottomRight());
        // Find diagonal pair top right bottom left
        std::vector<ColorPair> potential_pairs2 = findDiagonal(colors, robots.at(i).getTopRight(), robots.at(i).getBottomLeft());
        // See which pair of diagonals has the same center point
        findCorrectDiagonals(robots.at(i), potential_pairs1, potential_pairs2);
    }
}

void Main::findCorrectDiagonals(Robot robot, std::vector<ColorPair> pair1, std::vector<ColorPair> pair2) {
    for(int i = 0; i < pair1.size(); i++) {
        for(int j = 0; j < pair2.size(); j++) {
            float dist_x = pair1.at(i).getCenterX() - pair2.at(j).getCenterX();
            float dist_y = pair1.at(i).getCenterY() - pair2.at(j).getCenterY();
            float pyth = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
            if(pyth < DIAG_DIST_CENTER) {
                robot.found(true);
            } else {
                robot.found(false);
            }
        }
    }
}


std::vector<ColorPair> Main::findDiagonal(std::vector< std::vector<ColorLocation> > colors, std::string col1, std::string col2) {
    int col1_position = -1;
    int col2_position = -1;
    for(int i = 0; i < colors.size(); i++) {
        std::string currentColor = colors.at(i).at(0).getColor();
        if(currentColor == col1) {
            col1_position = i;
        } else if(currentColor == col2) {
            col2_position = i;
        }
    }

    std::vector<ColorPair> color_pair_list;

    if(col1_position != -1 && col2_position != -1) {
        for(int i =  0; i < colors.at(col1_position).size(); i++) {
            for(int j = 0; j < colors.at(col2_position).size(); j++) {
                if(inDiagonalRange(colors.at(col1_position).at(i), colors.at(col2_position).at(j))) {
                    ColorPair pair(colors.at(col1_position).at(i), colors.at(col2_position).at(j));
                    color_pair_list.push_back(pair);
                }
            }
        }
    }
}

bool Main::inDiagonalRange(ColorLocation col1, ColorLocation col2) {
    float x_dist = col1.getX() - col2.getX();
    float y_dist = col1.getY() - col2.getY();
    float pyth_dist = std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));
    if(pyth_dist > DIAG_DIST_MIN && pyth_dist < DIAG_DIST_MAX) {
        return true;
    } else {
        return false;
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