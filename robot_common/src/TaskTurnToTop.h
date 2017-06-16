//
// Created by montypylon on 6/14/17.
//

#ifndef PROJECT_TASKTURNTOTOP_H
#define PROJECT_TASKTURNTOTOP_H

#include <ros/ros.h>
#include <Robot.h>

class TaskTurnToTop {
    ros::Publisher command_pub;
    std::string last_cmd;
    double previousError;
    double integral;
    float previousTime;
    double Ku;
public:
    TaskTurnToTop(ros::NodeHandle);
    void action(Robot, double, float);
    double getPIDCorrection(double, double, double, int);
};


#endif //PROJECT_TASKTURNTOTOP_H
