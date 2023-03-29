#include "../include/nn/architure_initializer.h"
#include "../include/nn/networks/graph.h"
#include "../include/nn/weight_initializer.h"
#include "../include/nn/weight_optimizer.h"
#include "../include/utils.h"
#include "duckietown_msgs/WheelsCmdStamped.h"
#include "std_msgs/Bool.h"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
using namespace std::chrono;

using namespace cv;

class LaneControl {
public:
  static image_transport::Publisher pub;
  static ros::Publisher lane_control_pub;

  LaneControl() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    auto start = high_resolution_clock::now();

    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat cropped_image = img(cv::Range(330, 480), cv::Range(40, 150));

    int Hue_Lower_Value = 18;         // initial hue value(lower)//
    int Hue_Lower_Upper_Value = 41;   // initial hue value(upper)//
    int Saturation_Lower_Value = 66;  // initial saturation(lower)//
    int Saturation_Upper_Value = 210; // initial saturation(upper)//
    int Value_Lower = 100;            // initial value (lower)//
    int Value_Upper = 255;            // initial saturation(upper)//
    createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value,
                   179); // track-bar for lower hue//
    createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value,
                   179); // track-bar for lower-upper hue//
    createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value,
                   255); // track-bar for lower saturation//
    createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value,
                   255); // track-bar for higher saturation//
    createTrackbar("Val_Lower", "Adjust", &Value_Lower,
                   255); // track-bar for lower value//
    createTrackbar("Val_Upper", "Adjust", &Value_Upper,
                   255); // track-bar for upper value//

    Mat convert_to_HSV; // declaring a matrix to store converted image//
    cvtColor(cropped_image, convert_to_HSV,
             COLOR_BGR2HSV); // converting BGR image to HSV and storing it in
                             // convert_to_HSV matrix//
    Mat detection_screen; // declaring matrix for window where object will be
                          // detected//
    inRange(
        convert_to_HSV,
        Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower),
        Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper),
        detection_screen); // applying track-bar modified value of track-bar//

    //    erode(detection_screen, detection_screen,
    //    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological
    //    opening for removing small objects from foreground//
    //    dilate(detection_screen, detection_screen,
    //    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological
    //    opening for removing small object from foreground//
    //    dilate(detection_screen, detection_screen,
    //    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological
    //    closing for filling up small holes in foreground//
    //    erode(detection_screen, detection_screen,
    //    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological
    //    closing for filling up small holes in foreground//

    duckietown_msgs::WheelsCmdStamped control_msg;
    float avg_value = 0;
    for (int i = 0; i < cropped_image.rows; i++) {
      for (int j = 0; j < cropped_image.cols; j++) {
        cropped_image.at<cv::Vec3b>(i, j)[0] = detection_screen.at<uchar>(i, j);
        cropped_image.at<cv::Vec3b>(i, j)[1] = detection_screen.at<uchar>(i, j);
        cropped_image.at<cv::Vec3b>(i, j)[2] = detection_screen.at<uchar>(i, j);
        //        if(i == cropped_image.rows -1){
        //          std::cout << "Last row" << std::endl;
        //          std::cout << float(detection_screen.at<uchar>(i, j)) <<  " "
        //          << float(j)/float(cropped_image.cols) << std::endl;
        avg_value +=
            float(detection_screen.at<uchar>(i, j)) *
            ((float(float(j) < (4 * float(cropped_image.cols) / 5)) - 0.5) /
             float(cropped_image.cols));
        //        }
      }
    }
    //    std::cout << "avg val "  << avg_value << std::endl;

    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
            .toImageMsg();
    LaneControl::pub.publish(msg2);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //        std::cout << duration.count() << std::endl;

    //    control_msg.stamp = msg->stamp;
    bool right = false;
    //    bool straight = true;
    bool left = false;
    if (avg_value < 0)
      right = true;
    else if (avg_value > 0)
      left = true;

    //    bool straight = false;
    std::cout << avg_value << std::endl;
    if (right) {
      //      std::cout << "Turning right\n";
      //      std::cout << avg_value << std::endl;
      control_msg.vel_right = 0.02;
      control_msg.vel_left = 0.4;
    } else if (left) {
      //      std::cout << "Turning left\n";
      //      std::cout << avg_value << std::endl;
      control_msg.vel_right = 0.4;
      control_msg.vel_left = 0.02;
    } else {
      //      std::cout << "Goind straight\n";
      control_msg.vel_right = 0.2;
      control_msg.vel_left = 0.2;
    }
    //    control_msg.vel_right = 0;
    //    control_msg.vel_left = 0;
    LaneControl::lane_control_pub.publish(control_msg);
    //    std::cout << "Control Message published\n";
  }
};

void mySigintHandler(int sig) {
  duckietown_msgs::WheelsCmdStamped control_msg;
  control_msg.vel_right = 0.0;
  control_msg.vel_left = 0.0;
  LaneControl::lane_control_pub.publish(control_msg);
  ros::shutdown();
}

image_transport::Publisher LaneControl::pub;
ros::Publisher LaneControl::lane_control_pub;
int main(int argc, char **argv) {

  ros::init(argc, argv, "image_listener_control");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LaneControl::lane_control_pub =
      nh.advertise<duckietown_msgs::WheelsCmdStamped>(
          "/csc22940/wheels_driver_node/wheels_cmd", 1);
  image_transport::Subscriber sub = it.subscribe("/csc22940/camera_node/image",
                                                 1, LaneControl::imageCallback);
  LaneControl::pub = it.advertise("/lane_control/image", 1);
  signal(SIGINT, mySigintHandler);
  ros::spin();
}
