#include "ros/ros.h"
#include "std_msgs/String.h"
#include <robot_comm/Motor.h>
#include <sstream>
#include <signal.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

using namespace std;

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
    (void) sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void keyLoop(ros::Publisher pub) {
    char c;
    bool dirty = false;


    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");


    for (;;) {
        // get the next event from the keyboard  
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
        
        ROS_DEBUG("value: 0x%02X\n", c);
        //cout << "value of c: " << ("0x%02X", c) << endl;

        switch (c) {
            case KEYCODE_L: {
                robot_comm::Motor msgL;
                msgL.name = "Robot 1";
                msgL.left_power = -384;
                msgL.right_power = 384;
                pub.publish(msgL);
            } break;
            case KEYCODE_R: {
                robot_comm::Motor msgR;
                msgR.name = "Robot 1";
                msgR.left_power = 384;
                msgR.right_power = -384;
                pub.publish(msgR);
            } break;
            case KEYCODE_U: {
                robot_comm::Motor msgU;
                msgU.name = "Robot 1";
                msgU.left_power = 1023;
                msgU.right_power = 1023;
                pub.publish(msgU);
            } break;
            case KEYCODE_D: {
                robot_comm::Motor msgD;
                msgD.name = "Robot 1";
                msgD.left_power = -1023;
                msgD.right_power = -1023;
                pub.publish(msgD);
            } break;
            case 's': {
                robot_comm::Motor msgS;
                msgS.name = "Robot 1";
                msgS.left_power = 0;
                msgS.right_power = 0;
                pub.publish(msgS);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_main");
    ros::NodeHandle n;
    ros::Publisher command_pub = n.advertise<robot_comm::Motor>("/motor", 1000);
    signal(SIGINT, quit);
    keyLoop(command_pub);
    return 0;
}