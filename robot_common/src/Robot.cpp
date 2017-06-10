//
// Created by montypylon on 6/9/17.
//

#include "Robot.h"

using namespace std;

std::string Robot::getIP() {
    return ip;
}

void Robot::setColors(std::string top_left, std::string top_right, std::string bot_left, std::string bot_right) {
    top_left_color = top_left;
    top_right_color = top_right;
    bottom_left_color = bot_left;
    bottom_right_color = bot_right;
}

float Robot::getX() {
    return center_x;
}

float Robot::getY() {
    return center_y;
}