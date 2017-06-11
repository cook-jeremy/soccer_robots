//
// Created by montypylon on 6/9/17.
//

#include "Robot.h"

using namespace std;

std::string Robot::getIP() {
    return ip;
}

void Robot::setColors(std::string center, std::string top_left, std::string top_right, std::string bot_left, std::string bot_right) {
    center_color = center;
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

std::string Robot::getCenterColor() {
    return center_color;
}

std::string Robot::getTopLeftColor() {
    return top_left_color;
}

std::string Robot::getTopRightColor() {
    return top_right_color;
}

std::string Robot::getBottomLeftColor() {
    return bottom_left_color;
}

std::string Robot::getBottomRightColor() {
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