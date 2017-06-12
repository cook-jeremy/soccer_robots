//
// Created by montypylon on 5/13/17.
//

#include <ColorLocation.h>
//using namespace std;

ColorLocation::ColorLocation() {}
ColorLocation::ColorLocation(std::string col, float x1, float y1) {
    color = col;
    x = x1;
    y = y1;
}

std::string ColorLocation::getColor() {
    return color;
}

float ColorLocation::getX() {
    return x;
}

float ColorLocation::getY() {
    return y;
}

void ColorLocation::setX(float in_x) {
    x = in_x;
}

void ColorLocation::setY(float in_y) {
    y = in_y;
}

void ColorLocation::setColor(std::string in) {
    color = in;
}