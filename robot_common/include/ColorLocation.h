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
    int id;
public:
    ColorLocation();
    ColorLocation(std::string col, float x, float y);
    std::string getColor();
    float getX();
    float getY();
    void setX(float);
    void setY(float);
    void setColor(std::string);
    int getID();
    void setID(int);
};

#endif // __MYCLASS_H_INCLUDED__