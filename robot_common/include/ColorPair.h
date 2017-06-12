//
// Created by montypylon on 6/9/17.
//

#ifndef PROJECT_COLORPAIR_H
#define PROJECT_COLORPAIR_H

#include <string>
#include "ColorLocation.h"

class ColorPair {
    ColorLocation col1;
    ColorLocation col2;
    float center_x;
    float center_y;
public:
    ColorPair(ColorLocation, ColorLocation);
    float getCenterX();
    float getCenterY();

};

#endif //PROJECT_COLORPAIR_H
