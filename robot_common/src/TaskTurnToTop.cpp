//
// Created by montypylon on 6/14/17.
//

#include <std_msgs/String.h>
#include <robot_common/Motor.h>
#include "TaskTurnToTop.h"

TaskTurnToTop::TaskTurnToTop(ros::NodeHandle n) {
    command_pub = n.advertise<robot_common::Motor>("/motor", 1000);
    last_cmd = "";
}

void TaskTurnToTop::action(Robot robot, double desiredAngle) {
    double desired = desiredAngle * (3.141592653 / 180);
    double current = robot.getRadian();
    double delta = std::atan2(sin(desired-current), cos(desired-current));
    std::cout << "Delta is " << delta << std::endl;

    if(std::abs(delta) > 0.5) {
        //turn left when positive, turn right when negative
        if (delta > 0 && last_cmd != "l") {
            robot_common::Motor msgL;
            msgL.name = "Robot 1";
            msgL.left_power = -584;
            msgL.right_power = 584;
            command_pub.publish(msgL);
            last_cmd = "l";
        } else if (delta < 0 && last_cmd != "r") {
            robot_common::Motor msgR;
            msgR.name = "Robot 1";
            msgR.left_power = 584;
            msgR.right_power = -584;
            command_pub.publish(msgR);
            last_cmd = "r";
        }
    } else if(last_cmd != "s"){
        robot_common::Motor msgS;
        msgS.name = "Robot 1";
        msgS.left_power = 0;
        msgS.right_power = 0;
        command_pub.publish(msgS);
        last_cmd = "s";
    }

}