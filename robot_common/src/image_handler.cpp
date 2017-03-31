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
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
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

        /**
        //cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 255)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
         **/
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
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

        // DISCLAIMER: This code was borrowed from: http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
        using namespace cv;
        //declare output image
        cv::Mat original_image = cv_ptr->image;
        cv::Mat outImg;
        cv::Mat imgHSV;

        cvtColor(original_image, imgHSV, cv::COLOR_BGR2HSV); // Conver to HSV

        //cv::Mat imgThresholded;
        //cv::Mat autoThresh;
        cv::Mat autoThreshContour;

        /**
        //inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        inRange(imgHSV, cv::Scalar(MIN_HUE, MIN_SAT, MIN_VAL), cv::Scalar(MAX_HUE, MAX_SAT, MAX_VAL), autoThresh);
        cv::erode(autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate( autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate( autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        **/

        inRange(imgHSV, cv::Scalar(MIN_HUE, MIN_SAT, MIN_VAL), cv::Scalar(MAX_HUE, MAX_SAT, MAX_VAL), autoThreshContour);
        cv::erode(autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate( autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate( autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

        std::vector<std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;

        Mat temp = autoThreshContour.clone();
        /// Detect edges using canny
        //Canny( src_gray, canny_output, thresh, thresh*2, 3 );
        /// Find contours
        findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        std::vector<Moments> mu(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
        }

        ///  Get the mass centers:
        std::vector<Point2f> mc(contours.size());
        double maxArea = -1;
        double numMax = -1;
        double minArea = 20;
        for (int i = 0; i < contours.size(); i++) {
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
            double area = mu[i].m00;
            //std::cout << "Area of moment " << i << ": " << mu[i].m00 << std::endl;

            std::stringstream ss;
            ss << area;
            std::string areaStr = ss.str();

            //cv::putText(original_image, areaStr, mc[i], 1, 1, CV_RGB(0, 255, 255));

            if((area > maxArea) && (area > minArea)) {
                maxArea = area;
                numMax = i;
            }
        }

        double offset = 0;

        if(numMax != -1) {
            circle(original_image, mc[numMax], 5, CV_RGB(255, 255, 0), -1, 8, 0);

            double width = original_image.cols;
            offset = mc[numMax].x - (width/2);
            std::cout << "Offset: " << offset << std::endl;
            //Point2f middle = Point2f(original_image.cols / 2, original_image.rows / 2);
            //circle(original_image, middle, 5, CV_RGB(255, 255, 0), -1, 8, 0);
            //Point2f middle2 = Point2f(mc[numMax].x, original_image.rows / 2);
            //circle(original_image, middle2, 5, CV_RGB(255, 255, 0), -1, 8, 0);
        }

        //cv::imshow("Auto-Thresholded Image", autoThresh);
        cv::imshow("Auto-Thresholded Image", autoThreshContour);
        //cv::imshow("Thresholded Image", imgThresholded);
        cv::imshow("Original", original_image); //show the original image

        //pause for 3 ms
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;

    ros::spin();
    return 0;
}
