#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <ImageHandler.h>
#include <Robot.h>
#include <ColorLocation.h>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

double const PI = 3.141592653;

ImageHandler::ImageHandler(ros::NodeHandle n, image_transport::ImageTransport it_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageHandler::imageCb, this);

    //cv::namedWindow("Auto-Thresholded Image", CV_WINDOW_NORMAL);
    //cv::resizeWindow("Auto-Thresholded Image", 800, 600);
    //cv::moveWindow("Auto-Thresholded Image", 300, 300);

    cv::namedWindow("Original", CV_WINDOW_NORMAL | CV_GUI_NORMAL);
    cv::resizeWindow("Original", 800, 600);
    cv::moveWindow("Original", 300, 300);
    cvCreateButton("button1", ImageHandler::callbackButton, NULL, CV_PUSH_BUTTON, 0);
}


void ImageHandler::callbackButton(int state, void* userdata) {
    std::cout << "@my_button_cb" << std::endl;
}

ImageHandler::~ImageHandler() {
    cv::destroyWindow(OPENCV_WINDOW);
}

std::vector<ColorLocation> ImageHandler::getColorAndPosition(cv::Mat original_image, cv::Mat color_image, std::string colorName) {
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
    for (int i = 0; i < contours.size(); i++) {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        double area = mu[i].m00;
        if((area < MAX_AREA) && (area > MIN_AREA)) {
            found.push_back(i);
        }
    }

    std::vector<ColorLocation> group;

    if(!found.empty()) {
        for(int a = 0; a < found.size(); a++) {
            circle(original_image, mc[found.at(a)], 11, CV_RGB(51, 204, 166), 1, 8, 0);
            ColorLocation col(colorName, mc[found.at(a)].x, mc[found.at(a)].y);
            group.push_back(col);
        }
    }
    return group;
}

std::vector< std::vector<ColorLocation> > ImageHandler::getAllColors() {
    return all_colors;
}

void ImageHandler::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    all_colors.clear();

    using namespace cv;
    cv::Mat original_image = cv_ptr->image;
    img_ptr = cv_ptr;
    cv::Mat imgHSV;
    cvtColor(original_image, imgHSV, cv::COLOR_BGR2HSV); // Conver to HSV

    std::vector<cv::Mat> colors;
    cv::Mat red_img;
    cv::Mat blue_img;
    cv::Mat yellow_img;
    cv::Mat orange_img;
    std::vector<std::string> color_names;
    color_names.push_back("red");
    color_names.push_back("blue");
    color_names.push_back("yellow");
    color_names.push_back("orange");

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

    inRange(imgHSV, cv::Scalar(ORANGE_MIN_HUE, ORANGE_MIN_SAT, ORANGE_MIN_VAL), cv::Scalar(ORANGE_MAX_HUE, ORANGE_MAX_SAT, ORANGE_MAX_VAL), orange_img);
    cv::erode(orange_img, orange_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
    cv::dilate(orange_img, orange_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
    cv::dilate(orange_img, orange_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
    cv::erode(orange_img, orange_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

    colors.push_back(red_img);
    colors.push_back(blue_img);
    colors.push_back(yellow_img);
    colors.push_back(orange_img);

    //std::vector< std::vector<ColorLocation> > all_colors;

    for(int a = 0; a < NUM_COLORS; a++) {
        all_colors.push_back(getColorAndPosition(original_image, colors.at(a), color_names.at(a)));
    }

    //cv::imshow("Auto-Thresholded Image", orange_img);
    //pause for 3 ms
    //cv::waitKey(1);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}

cv_bridge::CvImagePtr ImageHandler::getImage() {
    return img_ptr;
}

void ImageHandler::drawCenter(cv_bridge::CvImagePtr img, Robot robot) {
    cv::Point2f center(robot.getX(), robot.getY());
    cv::circle(img->image, center, 4, CV_RGB(66, 235, 244), -1, 8, 0);
}

void ImageHandler::updateImage() {
    //cv::imshow("Auto-Thresholded Image", yellow_img);
    cv::imshow("Original", img_ptr->image); //show the original image
}
