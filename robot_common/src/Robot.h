//
// Created by montypylon on 6/9/17.
//

#ifndef PROJECT_ROBOT_H
#define PROJECT_ROBOT_H

#include <string>

class Robot {
    std::string name;
    std::string ip;
    std::string top_left_color;
    std::string top_right_color;
    std::string bottom_left_color;
    std::string bottom_right_color;
    float center_x;
    float center_y;
    float angle;
public:
    Robot(std::string _name, std::string _ip) : name(_name), ip(_ip) {}
    std::string getIP();
    void setColors(std::string, std::string, std::string, std::string);
    float getX();
    float getY();
};


#endif //PROJECT_ROBOT_H
