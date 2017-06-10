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

bool Robot::isLocated() {
    return located;
}

void Robot::found(bool isFound) {
    located = isFound;
}

std::string Robot::getTopLeft() {
    return top_left_color;
}

std::string Robot::getTopRight() {
    return top_right_color;
}

std::string Robot::getBottomLeft() {
    return bottom_left_color;
}

std::string Robot::getBottomRight() {
    return bottom_right_color;
}

float Robot::getX() {
    return center_x;
}

float Robot::getY() {
    return center_y;
}

void Robot::setX(float x) {
    center_x = x;
}

void Robot::setY(float y) {
    center_y = y;
}