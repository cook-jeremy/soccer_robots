#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Config.h"
#include "ColorPair.h"
#include "ImageHandler.h"
#include "TaskTurnToTop.h"
#include <time.h>
#include <sys/timeb.h>
//#include "Robot.h"

using namespace std;

class Main {
    bool first;
    bool turnToTop;
public:
    Main(ros::NodeHandle);
    std::vector<Robot> setupRobots();
    void findRobotPositions(std::vector<Robot> *robot_p, std::vector< std::vector<ColorLocation> >);
    std::vector<ColorPair> findDiagonal(std::vector< std::vector<ColorLocation> >, std::string, std::string);
    bool inDiagonalRange(ColorLocation, ColorLocation);
    void findCorrectDiagonals(Robot*, std::vector< std::vector<ColorLocation> >, std::vector<ColorPair>, std::vector<ColorPair>);
    bool isColorsEmpty(std::vector< std::vector<ColorLocation> >);
    void getAngle(Robot*, ColorPair, ColorPair);
    float getDotAngle(Robot*, ColorLocation);
    float calculateRobotAngle(Robot*, ColorLocation, ColorLocation);
    int getMilliCount();
    int getMilliSpan(int);
};

Main::Main(ros::NodeHandle n) {
    image_transport::ImageTransport it(n);
    ImageHandler ih(n, it);

    int current = getMilliCount();
    int milliSecondsElapsed;

    std::vector<Robot> robots = setupRobots();
    std::vector<Robot> * robot_p = &robots;
    first = true;
    int firstCounter = 0;

    TaskTurnToTop turn(n);
    TaskGoToCoordinates goto_coor(n);
    srand (time(NULL));
    double v1 = rand() % 360;
    cout << "Rand angle is: " << v1 << endl;

         cout << "Starting Main" << endl;
    while (ros::ok())
    {
        milliSecondsElapsed = getMilliSpan(current);
        current = getMilliCount();
        // Get all colors from the image
        std::vector< std::vector<ColorLocation> > colors = ih.getAllColors();
        cv_bridge::CvImagePtr img = ih.getImage();
        //cout << "all_colors" << colors.size() << endl;
        if(colors.size() != 0 && !first) {
            bool isEmpty = isColorsEmpty(colors);
            if(!isEmpty) {
                //cout << "x = " << colors.at(1).at(1).getX() << endl;
                findRobotPositions(robot_p, colors);
                for (int i = 0; i < robots.size(); i++) {
                    if (robots.at(i).isLocated()) {
                        ih.drawCenter(img, robots.at(i));
                        ih.drawDirection(img, robots.at(i));
                        turn.action(robots.at(i), v1, milliSecondsElapsed);
                    }
                }
            }
        }
        colors.clear();
        if(img) {
            cv::imshow("Original", img->image);
        }
        if(firstCounter < 20) {
            firstCounter++;
        } else {
            first = false;
        }
        cv::waitKey(1);
        ros::spinOnce();
    }
}

int Main::getMilliCount() {
    timeb tb;
    ftime(&tb);
    int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
    return nCount;
}

int Main::getMilliSpan(int nTimeStart){
    int nSpan = getMilliCount() - nTimeStart;
    if(nSpan < 0)
        nSpan += 0x100000 * 1000;
    return nSpan;
}

bool Main::isColorsEmpty(std::vector< std::vector<ColorLocation> > colors) {
    int j = 0;
    int size = colors.size();
    for(int i = 0; i < colors.size(); i++) {
        if(colors.at(i).size() == 0) {
            j++;
        }
    }
    if(j == size) {
        return true;
    } else {
        return false;
    }
}

void Main::findRobotPositions(std::vector<Robot> *robots, std::vector< std::vector<ColorLocation> > colors) {
    if(!robots->empty()) {
        for (int i = 0; i < robots->size(); i++) {
            // Find diagonal pair top left bottom right
            std::vector<ColorPair> potential_pairs1 = findDiagonal(colors, robots->at(i).getTopLeftColor(), robots->at(i).getBottomRightColor());
            // Find diagonal pair top right bottom left
            std::vector<ColorPair> potential_pairs2 = findDiagonal(colors, robots->at(i).getTopRightColor(), robots->at(i).getBottomLeftColor());
            // See which pair of diagonals has the same center point
            findCorrectDiagonals(&robots->at(i), colors, potential_pairs1, potential_pairs2);
        }
    }
}

void Main::findCorrectDiagonals(Robot *robot, std::vector< std::vector<ColorLocation> > colors, std::vector<ColorPair> pair1, std::vector<ColorPair> pair2) {
    for(int i = 0; i < pair1.size(); i++) {
        for(int j = 0; j < pair2.size(); j++) {
            float dist_x = pair1.at(i).getCenterX() - pair2.at(j).getCenterX();
            float dist_y = pair1.at(i).getCenterY() - pair2.at(j).getCenterY();
            float pyth = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
            float robot_center_x = pair1.at(i).getCenterX() - ((pair1.at(i).getCenterX() - pair2.at(j).getCenterX()) / 2);
            float robot_center_y = pair1.at(i).getCenterY() - ((pair1.at(i).getCenterY() - pair2.at(j).getCenterY()) / 2);

            int color_position = -1;
            for(int k = 0; k < colors.size(); k++) {
                if(colors.at(k).size() != 0) {
                    std::string currentColor = colors.at(k).at(0).getColor();
                    if (currentColor == robot->getCenterColor()) {
                        color_position = k;
                        continue;
                    }
                }
            }

            bool color_at_center = false;
            for(int l = 0; l < colors.at(color_position).size(); l++) {
                float color_x = colors.at(color_position).at(l).getX();
                float color_y = colors.at(color_position).at(l).getY();
                float diff_x = color_x - robot_center_x;
                float diff_y = color_y - robot_center_y;
                float pyth2 = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
                if(pyth2 < DIAG_DIST_CENTER) {
                    color_at_center = true;
                    continue;
                }
            }

            if(pyth < DIAG_DIST_CENTER && color_at_center) {
                robot->found(true);
                robot->setX(robot_center_x);
                robot->setY(robot_center_y);

                getAngle(robot, pair1.at(i), pair2.at(j));

                return;
            }
        }
    }
    robot->found(false);
}

void Main::getAngle(Robot *robot, ColorPair pair1, ColorPair pair2) {
    std::vector<ColorLocation> four_colors;
    four_colors.push_back(pair1.getColorLocation1());
    four_colors.push_back(pair1.getColorLocation2());
    four_colors.push_back(pair2.getColorLocation1());
    four_colors.push_back(pair2.getColorLocation2());
    std::vector<float> anglesBefore;
    std::vector<float> anglesAfter;
    for(int i = 0; i < four_colors.size(); i++) {
        float angle = getDotAngle(robot, four_colors.at(i));
        anglesBefore.push_back(angle);
        anglesAfter.push_back(angle);
    }
    std::sort(anglesAfter.begin(), anglesAfter.end());
    std::vector<int> after;
    for(int a = 0; a < 4; a++) {
        for(int b = 0; b < 4; b++) {
            if(anglesAfter.at(a) == anglesBefore.at(b)) {
                after.push_back(b);
            }
        }
    }
    std::vector<ColorLocation> anticlockwise;
    for(int c = 0; c < 4; c++) {
        anticlockwise.push_back(four_colors.at(after.at(c)));
    }
    std::vector<int> mylist;
    mylist.push_back(0);
    mylist.push_back(1);
    mylist.push_back(2);
    mylist.push_back(3);
    for(int d = 0; d < 4; d++) {
        if(anticlockwise.at(mylist.at(0)).getColor() == robot->getTopLeftColor() &&
                anticlockwise.at(mylist.at(1)).getColor() == robot->getBottomLeftColor() &&
                anticlockwise.at(mylist.at(2)).getColor() == robot->getBottomRightColor() &&
                anticlockwise.at(mylist.at(3)).getColor() == robot->getTopRightColor()) {
            robot->setAngle(calculateRobotAngle(robot, anticlockwise.at(mylist.at(0)), anticlockwise.at(mylist.at(3))));
        }
        std::rotate(mylist.begin(), mylist.begin() + 1, mylist.end());
    }
    int test = 0;
}

float Main::calculateRobotAngle(Robot *robot, ColorLocation tl, ColorLocation tr) {
    float rx = robot->getX();
    float ry = robot->getY();
    float cx = tl.getX() - ((tl.getX() - tr.getX())/2);
    float cy = tl.getY() - ((tl.getY() - tr.getY())/2);
    float x = cx - rx;
    float y = ry - cy;
    float angle = std::atan(y/x);
    if(x < 0 && y > 0 || x < 0 && y < 0) {
        angle += 3.141592653;
    } else if(x > 0 && y < 0) {
        angle += 2 * 3.141592653;
    }
    angle *= (180/3.141592653);
    return angle;
}

float Main::getDotAngle(Robot *robot, ColorLocation potential) {
    float y = robot->getY() - potential.getY();
    float x = potential.getX() - robot->getX();
    float angle = std::atan(y/x);
    if(x < 0 && y > 0 || x < 0 && y < 0) {
        angle += 3.141592653;
    } else if(x > 0 && y < 0) {
        angle += 2 * 3.141592653;
    }
    return angle;
}

std::vector<ColorPair> Main::findDiagonal(std::vector< std::vector<ColorLocation> > colors, std::string col1, std::string col2) {
    int col1_position = -1;
    int col2_position = -1;
    for(int i = 0; i < colors.size(); i++) {
        if(colors.at(i).size() != 0) {
            std::string currentColor = colors.at(i).at(0).getColor();
            if (currentColor == col1 && currentColor == col2) {
                col1_position = i;
                col2_position = i;
                continue;
            } else if (currentColor == col1) {
                col1_position = i;
            } else if (currentColor == col2) {
                col2_position = i;
            }
        }
    }

    std::vector<ColorPair> color_pair_list;

    if(col1_position != -1 && col2_position != -1) {

        if(col1_position == col2_position) {
            for(int i = 0; i < colors.at(col1_position).size(); i++) {
                for (int j = 0; j < colors.at(col2_position).size(); j++) {
                    if(i != j) {
                        if (inDiagonalRange(colors.at(col1_position).at(i), colors.at(col2_position).at(j))) {
                            ColorPair pair(colors.at(col1_position).at(i), colors.at(col2_position).at(j));
                            color_pair_list.push_back(pair);
                        }
                    }
                }
            }
        } else {
            for (int i = 0; i < colors.at(col1_position).size(); i++) {
                for (int j = 0; j < colors.at(col2_position).size(); j++) {
                    if (inDiagonalRange(colors.at(col1_position).at(i), colors.at(col2_position).at(j))) {
                        ColorPair pair(colors.at(col1_position).at(i), colors.at(col2_position).at(j));
                        color_pair_list.push_back(pair);
                    }
                }
            }
        }
    }
    return color_pair_list;
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
    robot1.setColors(ROBOT_1_CENTER, ROBOT_1_TOP_LEFT, ROBOT_1_TOP_RIGHT, ROBOT_1_BOTTOM_LEFT, ROBOT_1_BOTTOM_RIGHT);
    robots.push_back(robot1);

    Robot robot3(ROBOT_3_NAME, ROBOT_3_IP);
    robot3.setColors(ROBOT_3_CENTER, ROBOT_3_TOP_LEFT, ROBOT_3_TOP_RIGHT, ROBOT_3_BOTTOM_LEFT, ROBOT_3_BOTTOM_RIGHT);
    robots.push_back(robot3);
    return robots;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    Main main(n);
    ros::spin();
    return 0;
}