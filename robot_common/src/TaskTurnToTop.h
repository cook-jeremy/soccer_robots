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
public:
    TaskTurnToTop(ros::NodeHandle);
    void action(Robot, double);
};


#endif //PROJECT_TASKTURNTOTOP_H
