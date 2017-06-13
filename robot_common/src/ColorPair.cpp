//
// Created by montypylon on 6/9/17.
//

#include "ColorPair.h"

ColorPair::ColorPair(ColorLocation color1, ColorLocation color2) : col1(color1), col2(color2) {
    center_x = col1.getX() - ((col1.getX() - col2.getX()) / 2);
    center_y = col1.getY() - ((col1.getY() - col2.getY()) / 2);
}

float ColorPair::getCenterX() {
    return center_x;
}

float ColorPair::getCenterY() {
    return center_y;
}

std::string ColorPair::getColor1() {
    return col1.getColor();
}

std::string ColorPair::getColor2() {
    return col2.getColor();
}

ColorLocation ColorPair::getColorLocation1() {
    return col1;
}

ColorLocation ColorPair::getColorLocation2() {
    return col2;
}