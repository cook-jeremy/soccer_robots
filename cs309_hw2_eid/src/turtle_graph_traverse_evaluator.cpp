#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

#include <cs309_turtlesim/turtle_constants.h>

//global publisher for velocity
ros::Publisher velocity_pub;

//the turtle's current pose gets stored here
turtlesim::Pose current_pose;

//flag that gets set to true if the turtle starts moving
bool started_moving = false;

//vector of boolean that keeps track of which waypoints have been visited
std::vector<bool> locations_status;
int num_visited = 0;

double t_start = 0.0;
double t_end = 0.0;
double distance_traveled = 0.0;

turtlesim::Pose p;
bool have_last_p = false;


double euc_distance(double x1, double y1, double x2, double y2){
	return sqrt( (x1- x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) );	
}
	

void pose_cb(const turtlesim::Pose::ConstPtr& msg){
	current_pose = *msg;
	
	//compute traveled distance from last time message was received
	if (have_last_p){
		double d = sqrt( (msg->x - p.x)*(msg->x - p.x) + (msg->y - p.y)*(msg->y - p.y) );
		distance_traveled += d;
	}
	
	//update last pose
	p = *msg;
	have_last_p = true;
	
	//check to see if any location is visited
	for (unsigned int i = 0; i < locations_status.size(); i ++){
		if (locations_status[i] == false){ //only check if location i has not been visited
			double d_i = euc_distance(msg->x, msg->y, x_coords[i],y_coords[i]);
			if (d_i < DISTANCE_THRESHOLD){
				ROS_INFO("Visited location %i!",i);
				locations_status[i] = true;
				num_visited ++;
				
				if (num_visited == NUM_WAYPOINTS){
					t_end = ros::Time::now().toSec();
					ROS_INFO("Finished in %f seconds after traveling %f turtle meters!",t_end-t_start,distance_traveled);
					ros::shutdown();
				}
			}
		}
	}
	
	//EXTRA CREDIT: if a location is visited, make a service call to "kill" the turtle corresponding to that location
}

void vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
	if (started_moving == false){
		ROS_INFO("Movement started!");
		started_moving = true;
		
		t_start = ros::Time::now().toSec();
	}
	
	
	//check to make sure linear and angular velocities are within limits
	bool cheated = false;
	
	
	if (msg->linear.x > MAX_LINEAR_VELOCITY){
		ROS_WARN("Detected linear velocity greater than %f.",msg->linear.x);
		cheated = true;
	}
	else if (msg->linear.x < 0){
		ROS_WARN("Detected linear velocity less than 0.");
		cheated = true;
	}
	
	if (msg->angular.z > MAX_ANGULAR_VELOCITY){
		ROS_WARN("Detected angular velocity greater than %f.",msg->angular.z);
		cheated = true;
	}
	
	if (msg->linear.y > 0 || msg->linear.z > 0){
		ROS_WARN("Linear y and z velocities must be 0.");
		cheated = true;
	}
	
	if (cheated){
		ROS_ERROR("Cheating detected! Address the issue and try again...");
	}
}

int main(int argc, char **argv){
	
	//initialize the node
	ros::init(argc, argv, "turtle_graph_evaluator");
	
	//instantiate the node handle which is used for creating publishers and subscribers
	ros::NodeHandle n;
	
	//subscriber for the turtle's pose
	ros::Subscriber sub_pose = n.subscribe("/turtle1/pose", 1, pose_cb );
   
	//subscriber for the turtle's pose
	ros::Subscriber sub_cmd_vel = n.subscribe("/turtle1/cmd_vel", 1, vel_cb );
   
	//service client for spawning turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawn_client = n.serviceClient<turtlesim::Spawn>("spawn");
	
   
   
	//initialize vector keeping track of locations
	for (int i = 0; i < NUM_WAYPOINTS; i ++){
		locations_status.push_back(false);
		
		//for each location, spawn a turtle
		turtlesim::Spawn srv_i;
		srv_i.request.x = x_coords[i];
		srv_i.request.y = y_coords[i];
	
		bool result_i = spawn_client.call(srv_i);
		if (!result_i){
			ROS_ERROR("Failed at spawning turtle!");
		}
	}
  
	ros::spin();
	
	return 0;
}
