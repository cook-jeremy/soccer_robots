#include "ros/ros.h"

#define PI 3.14159265359

//your turtle is obly allowed to moving in the x direction with the 
//following maximum velocity:
#define MAX_LINEAR_VELOCITY 1.0

//when turtning around the z-axix, the turtle is only allowed to turn this fast
#define MAX_ANGULAR_VELOCITY 0.5

//how many points
#define NUM_WAYPOINTS 10

//how close the turtle needs to get to a location
#define DISTANCE_THRESHOLD 0.5

//the x and y coordinates of the points the turtle has to visit
//these are C++ arrays. You can access them by x_coords[i] where
//i is an integer in the range of 0-9. 
double x_coords[] = {1.3,1.0,4.5,5.5,2.3,6.6,8.6,7.8,3.7,8.7};
double y_coords[] = {6.5,2.1,9.8,3.4,7.8,8.9,2.1,8.1,2.3,4.7};

	
		
		
		
