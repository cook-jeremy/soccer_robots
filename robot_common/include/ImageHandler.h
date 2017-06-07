//
// Created by montypylon on 6/6/17.
//

#ifndef PROJECT_IMAGEHANDLER_H
#define PROJECT_IMAGEHANDLER_H

//=================================
// forward declared dependencies
class ImageHandler;
//=================================

#include "ros/ros.h"


class ImageHandler {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
public:
    std::vector< std::vector<int> > getCoordinates(cv::Mat original_image, cv::Mat color_image);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
};

#endif //PROJECT_IMAGEHANDLER_H
