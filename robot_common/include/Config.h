#ifndef __CONFIG_H_INCLUDED__
#define __CONFIG_H_INCLUDED__


#define ROBOT_1_NAME "Lisa"
#define ROBOT_1_IP "192.168.1.201"
#define ROBOT_1_TOP_LEFT "blue"
#define ROBOT_1_TOP_RIGHT "yellow"
#define ROBOT_1_BOTTOM_LEFT "red"
#define ROBOT_1_BOTTOM_RIGHT "blue"

std::string ROBOT_2_NAME = "";
std::string ROBOT_2_IP = "192.168.1.202";
std::string ROBOT_2_TOP_LEFT = "";
std::string ROBOT_2_TOP_RIGHT = "";
std::string ROBOT_2_BOTTOM_LEFT = "";
std::string ROBOT_2_BOTTOM_RIGHT = "";

std::string ROBOT_3_NAME = "";
std::string ROBOT_3_IP = "192.168.1.203";
std::string ROBOT_3_TOP_LEFT = "";
std::string ROBOT_3_TOP_RIGHT = "";
std::string ROBOT_3_BOTTOM_LEFT = "";
std::string ROBOT_3_BOTTOM_RIGHT = "";

#define NUM_COLORS = 3;

#define RED_MIN_HUE 0
#define RED_MAX_HUE 9
#define RED_MIN_SAT 176
#define RED_MAX_SAT 255
#define RED_MIN_VAL 85
#define RED_MAX_VAL 255

#define BLUE_MIN_HUE 87
#define BLUE_MAX_HUE 131
#define BLUE_MIN_SAT 3
#define BLUE_MAX_SAT 255
#define BLUE_MIN_VAL 55
#define BLUE_MAX_VAL 255

#define YELLOW_MIN_HUE 16
#define YELLOW_MAX_HUE 26
#define YELLOW_MIN_SAT 135
#define YELLOW_MAX_SAT 255
#define YELLOW_MIN_VAL 39
#define YELLOW_MAX_VAL 255

#define MIN_AREA 50
#define MAX_AREA 600
#define DIAG_DIST_MIN 45
#define DIAG_DIST_MAX 85
#define DIAG_DIST_CENTER 15

#endif