//
// Created by montypylon on 5/13/17.
//

#include "ColorLocation.h"
using namespace std;

class ColorLocation {
    int width, height;
    public:
        ColorLocation(int, int);
        int area() { return (width*height); }
};

ColorLocation::ColorLocation(int a, int b) {
    width = a;
    height = b;
}


string ColorLocation::getColor() {
    return color;
}

vector<int> ColorLocation::getPos() {
    vector<int> xy;
    xy.push_back(x);
    xy.push_back(y);
    return xy;
}