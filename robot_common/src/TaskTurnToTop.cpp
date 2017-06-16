//
// Created by montypylon on 6/14/17.
//

#include <std_msgs/String.h>
#include <robot_common/Motor.h>
#include "TaskTurnToTop.h"

TaskTurnToTop::TaskTurnToTop(ros::NodeHandle n) {
    command_pub = n.advertise<robot_common::Motor>("/motor", 1000);
    last_cmd = "";
    previousError = 0;
    integral = 0;
    Ku = 3;
}

void TaskTurnToTop::action(Robot robot, double desiredAngle, float time) {
    double desired = desiredAngle * (3.141592653 / 180);
    double current = robot.getRadian();
    double delta = std::atan2(sin(desired-current), cos(desired-current));
    std::cout << "Delta is " << delta << std::endl;
    std::cout << "Time elapsed: " << time << std::endl;
    double minPower = 250;
    double maxPower = 700;

    if(std::abs(delta) > 0.1) {
        //turn left when positive, turn right when negative
        double output = getPIDCorrection(delta, minPower, maxPower, time);
        std::cout << "PID: " << output << std::endl;
        output;
        if(output < minPower) {
            output = minPower;
        } else if (output > maxPower) {
            output = maxPower;
        }
        std::cout << "Power: " << output << std::endl;
        if(delta < 0 && last_cmd != "l") {
            robot_common::Motor msgL;
            msgL.name = "Robot 1";
            msgL.left_power = (int)output;
            msgL.right_power = -(int)output;
            command_pub.publish(msgL);
            last_cmd = "l";
        } else if(delta > 0 && last_cmd != "r") {
            robot_common::Motor msgR;
            msgR.name = "Robot 1";
            msgR.left_power = -(int)output;
            msgR.right_power = (int)output;
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

double TaskTurnToTop::getPIDCorrection(double error, double minPower, double maxPower, int dt) {
    //Ku -= 0.001;
    double T = 5;
    double Kp = Ku*0.5;
    //double Ki = Kp*2/T;
    double Ki = 0;
    //double Kd = Kp*T/40;
    double Kd = 0;
    std::cout << "Ku = " << Ku << std::endl;
    std::cout << "Kp = " << Kp << std::endl;
    std::cout << "Ki = " << Ki << std::endl;
    std::cout << "Kd = " << Kd << std::endl;
    integral = integral + error*dt;
    double derivative = (error - previousError)/dt;
    double output = Kp*error + Ki*integral + Kd*derivative;
    previousError = error;
    return output;
}