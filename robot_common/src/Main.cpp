#include <ros/ros.h>
#include <std_msgs/String.h>
#include "ImageHandler.h"
#include "ColorLocation.h"
#include <image_transport/image_transport.h>


using namespace std;

class Main {
public:
    Main(ros::NodeHandle);
    void setupRobots();
};

Main::Main(ros::NodeHandle n) {
    image_transport::ImageTransport it(n);
    ImageHandler ih(n, it);

    setupRobots();

    cout << "Starting Main" << endl;
    while (ros::ok())
    {
        // Get all colors from the image
        std::vector< std::vector<ColorLocation> > colors = ih.getAllColors();

        ros::spinOnce();
    }
}

void Main::setupRobots() {

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    Main main(n);
    ros::spin();
    return 0;
}