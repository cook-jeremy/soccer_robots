//
// Created by montypylon on 6/6/17.
//

#ifndef PROJECT_IMAGEHANDLER_H
#define PROJECT_IMAGEHANDLER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Config.h>
#include <string>
#include <tf/tf.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <ImageHandler.h>
#include "geometry_msgs/PoseArray.h"
#include "ColorLocation.h"
#include "Robot.h"

class ImageHandler {
    image_transport::Subscriber image_sub_;
    std::vector< std::vector<ColorLocation> > all_colors;
    cv_bridge::CvImagePtr img_ptr;
public:
    ImageHandler(ros::NodeHandle n, image_transport::ImageTransport it);
    ~ImageHandler();
    std::vector<ColorLocation> getColorAndPosition(cv::Mat, cv::Mat, std::string);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    std::vector< std::vector<ColorLocation> > getAllColors();
    void drawCenter(cv_bridge::CvImagePtr, Robot);
    cv_bridge::CvImagePtr getImage();
    void updateImage();
};

#endif //PROJECT_IMAGEHANDLER_H
