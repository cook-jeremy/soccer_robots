//=================================
// include guard
#ifndef __COLORLOCATION_H_INCLUDED__
#define __COLORLOCATION_H_INCLUDED__

#include <string>

class ColorLocation
{
    std::string color;
    float x;
    float y;
public:
    ColorLocation(std::string col, float x, float y);
    std::string getColor();
    float getX();
    float getY();
};

#endif // __MYCLASS_H_INCLUDED__