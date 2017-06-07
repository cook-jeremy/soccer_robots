//=================================
// include guard
#ifndef __COLORLOCATION_H_INCLUDED__
#define __COLORLOCATION_H_INCLUDED__

//=================================
// forward declared dependencies
class ColorLocation;
//=================================
// included dependencies
#include "ros/ros.h"
#include <string.h>
#include <vector>
using namespace std;
//=================================
// the actual class
class ColorLocation {
    string color;
    int x;
    int y;

    public:
        ColorLocation(int, int);
        string getColor();
        vector<int> getPos();
};

#endif // __MYCLASS_H_INCLUDED__