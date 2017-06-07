#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include <config.h>
#include <string>
#include <tf/tf.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseArray.h"

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

int iLowH = 0;
int iHighH = 179;
int iLowS = 0;
int iHighS = 255;
int iLowV = 0;
int iHighV = 255;

double const PI = 3.141592653;

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
            : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
        //image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow("Auto-Thresholded Image", CV_WINDOW_NORMAL);
        cv::resizeWindow("Auto-Thresholded Image", 800, 600);
        cv::moveWindow("Auto-Thresholded Image", 300, 300);

        cv::namedWindow("Original", CV_WINDOW_NORMAL);
        cv::resizeWindow("Original", 800, 600);
        cv::moveWindow("Original", 300, 300);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    std::vector< std::vector<int> > getCoordinates(cv::Mat original_image, cv::Mat color_image) {
        using namespace cv;
        std::vector< std::vector<int> > pos;
        std::vector< std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;

        Mat temp = color_image.clone();
        findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        std::vector<Moments> mu(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
        }

        ///  Get the mass centers:
        std::vector<Point2f> mc(contours.size());
        std::vector<int> found;
        double numMax = -1;
        int maxFound = -1;
        for (int i = 0; i < contours.size(); i++) {
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
            double area = mu[i].m00;
            if((area < MAX_AREA) && (area > MIN_AREA)) {
                found.push_back(i);
            }
        }

        std::vector<int> posXY;
        if(!found.empty()) {
            for(int a = 0; a < found.size(); a++) {
                circle(original_image, mc[found.at(a)], 11, CV_RGB(51, 204, 166), 1, 8, 0);
                posXY.push_back(mc[numMax].x);
                posXY.push_back(mc[numMax].y);
                pos.push_back(posXY);
            }
        }

        return pos;
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        using namespace cv;
        cv::Mat original_image = cv_ptr->image;
        cv::Mat imgHSV;
        cvtColor(original_image, imgHSV, cv::COLOR_BGR2HSV); // Conver to HSV

        std::vector<cv::Mat> colors;
        cv::Mat red_img;
        cv::Mat blue_img;
        cv::Mat yellow_img;

        inRange(imgHSV, cv::Scalar(RED_MIN_HUE, RED_MIN_SAT, RED_MIN_VAL), cv::Scalar(RED_MAX_HUE, RED_MAX_SAT, RED_MAX_VAL), red_img);
        cv::erode(red_img, red_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate(red_img, red_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate(red_img, red_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(red_img, red_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

        inRange(imgHSV, cv::Scalar(BLUE_MIN_HUE, BLUE_MIN_SAT, BLUE_MIN_VAL), cv::Scalar(BLUE_MAX_HUE, BLUE_MAX_SAT, BLUE_MAX_VAL), blue_img);
        cv::erode(blue_img, blue_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate(blue_img, blue_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate(blue_img, blue_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(blue_img, blue_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

        inRange(imgHSV, cv::Scalar(YELLOW_MIN_HUE, YELLOW_MIN_SAT, YELLOW_MIN_VAL), cv::Scalar(YELLOW_MAX_HUE, YELLOW_MAX_SAT, YELLOW_MAX_VAL), yellow_img);
        cv::erode(yellow_img, yellow_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate(yellow_img, yellow_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate(yellow_img, yellow_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(yellow_img, yellow_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

        colors.push_back(red_img);
        colors.push_back(blue_img);
        colors.push_back(yellow_img);

        std::vector< std::vector<int> > all_color_pos;

        //for(int a = 0; a < NUM_COLORS; a++) {
            //all_color_pos.push_back(getCoordinates(original_image, colors.at(a)));
        //}

        cv::imshow("Auto-Thresholded Image", blue_img);
        cv::imshow("Original", original_image); //show the original image
        //pause for 3 ms
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

/**
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;

    ros::spin();
    return 0;
}
 **/
