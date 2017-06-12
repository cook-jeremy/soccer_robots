//
// Created by montypylon on 6/9/17.
//

#ifndef PROJECT_ROBOT_H
#define PROJECT_ROBOT_H

#include <string>
#include "ColorLocation.h"

class Robot {
    std::string name;
    std::string ip;
    std::string top_left_color;
    std::string top_right_color;
    std::string bottom_left_color;
    std::string bottom_right_color;
    std::string center_color;
    bool located;
    float center_x;
    float center_y;
    float angle;
    ColorLocation topLeft();
    ColorLocation topRight();
    ColorLocation bottomLeft();
    ColorLocation bottomRight();
public:
    Robot(std::string _name, std::string _ip) : name(_name), ip(_ip) {located = false;}
    std::string getIP();
    void setColors(std::string, std::string, std::string, std::string, std::string);
    float getX();
    float getY();
    void setX(float);
    void setY(float);
    void found(bool);
    bool isLocated();
    std::string getTopLeftColor();
    std::string getTopRightColor();
    std::string getBottomLeftColor();
    std::string getBottomRightColor();
    std::string getCenterColor();
};


#endif //PROJECT_ROBOT_H
