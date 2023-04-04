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

extern "C" {
#include "apriltag.h"
#include "common/getopt.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

using namespace cv;

class LaneControl {
public:
  static image_transport::Publisher pub;
  static image_transport::Publisher april_tag_pub;
  static ros::Publisher lane_control_pub;
  static float old_centroid;
  static int global_t;
  static apriltag_detector_t *td;
  static apriltag_family_t *tf;
  static ros::Time last_detection;
  static std::vector<int> detected;
  LaneControl() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    LaneControl::global_t++;
    auto start = high_resolution_clock::now();

    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    //    if(LaneControl::global_t%60==0){
    //      cv::imwrite("/data/image" + std::to_string(LaneControl::global_t)
    //      +".jpg", img);
    //    }
    cv::Mat cropped_image = img(cv::Range(230, 480), cv::Range(0, 640));
    cv::Mat cropped_image_above = img(cv::Range(0, 250), cv::Range(110, 530));

    cv::Mat blackandwhite;
    cv::cvtColor(cropped_image_above, blackandwhite, cv::COLOR_BGR2GRAY);

    image_u8_t img_header = {.width = blackandwhite.cols,
                             .height = blackandwhite.rows,
                             .stride = blackandwhite.cols,
                             .buf = blackandwhite.data};
    zarray_t *detections =
        apriltag_detector_detect(LaneControl::td, &img_header);

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      float dif = std::abs(det->p[1][1] - det->p[3][1]);
      float dif2 = std::abs(det->p[1][0] - det->p[3][0]);
      float area = dif * dif2;
      if ((ros::Time::now() - LaneControl::last_detection).toSec() > 3 &&
          area > 2500) {
        //        LaneControl::detected.push_back(int(det->id));
        std::cout
            << "Tag " << int(det->id)
            << " detected for the first time; taking appropriate action\n";
        LaneControl::last_detection = ros::Time::now();
      }

      //      std::cout << "Apriltag detected = " << int(det->id)  << " area "
      //      << area << std::endl;
    }

    sensor_msgs::ImagePtr msg3 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image_above)
            .toImageMsg();
    LaneControl::april_tag_pub.publish(msg3);

    Mat convert_to_HSV; // declaring a matrix to store converted image//
    cvtColor(cropped_image, convert_to_HSV,
             COLOR_BGR2HSV); // converting BGR image to HSV and storing it in
                             // convert_to_HSV matrix//

    int Hue_Lower_Value = 18;         // initial hue value(lower)//
    int Hue_Lower_Upper_Value = 41;   // initial hue value(upper)//
    int Saturation_Lower_Value = 66;  // initial saturation(lower)//
    int Saturation_Upper_Value = 210; // initial saturation(upper)//
    int Value_Lower = 100;            // initial value (lower)//
    int Value_Upper = 255;            // initial saturation(upper)//

    Mat yellow_lane_detection; // declaring matrix for window where object will
                               // be detected//
    inRange(convert_to_HSV,
            Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower),
            Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper),
            yellow_lane_detection); // applying track-bar modified value of
                                    // track-bar//

    Hue_Lower_Value = 46;        // initial hue value(lower)//
    Hue_Lower_Upper_Value = 171; // initial hue value(upper)//
    Saturation_Lower_Value = 0;  // initial saturation(lower)//
    Saturation_Upper_Value = 82; // initial saturation(upper)//
    Value_Lower = 174;           // initial value (lower)//
    Value_Upper = 255;           // initial saturation(upper)//

    Mat while_lane_detection; // declaring matrix for window where object will
                              // be detected//
                              //
                              //    cvtColor(cropped_image, convert_to_HSV,
    //             COLOR_BGR2HSV); // converting BGR image to HSV and storing it
    //             in
    //                             // convert_to_HSV matrix//
    inRange(convert_to_HSV,
            Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower),
            Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper),
            while_lane_detection); // applying track-bar modified value of
                                   // track-bar//

    Hue_Lower_Value = 0;          // initial hue value(lower)//
    Hue_Lower_Upper_Value = 20;   // initial hue value(upper)//
    Saturation_Lower_Value = 98;  // initial saturation(lower)//
    Saturation_Upper_Value = 255; // initial saturation(upper)//
    Value_Lower = 178;            // initial value (lower)//
    Value_Upper = 255;            // initial saturation(upper)//

    Mat duck_detection; // declaring matrix for window where object will
                        // be detected//

    inRange(convert_to_HSV,
            Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower),
            Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper),
            duck_detection); // applying track-bar modified value of
                             // track-bar//

    duckietown_msgs::WheelsCmdStamped control_msg;
    float avg_value = 0;
    float yellow_centroid = 0;
    float white_centroid = 0;
    float yellow_total = 0;
    float white_total = 0;
    float centroid_white;
    for (int i = 0; i < cropped_image.rows; i++) {
      for (int j = 0; j < cropped_image.cols; j++) {
        cropped_image.at<cv::Vec3b>(i, j)[0] = duck_detection.at<uchar>(i, j);
        cropped_image.at<cv::Vec3b>(i, j)[1] =
            yellow_lane_detection.at<uchar>(i, j);
        cropped_image.at<cv::Vec3b>(i, j)[2] =
            while_lane_detection.at<uchar>(i, j);

        if ((float(yellow_lane_detection.at<uchar>(i, j)) * float(j) /
             float(cropped_image.cols)) > 255 * 0.8) {
          //          std::cout << "Greter than that "
          //                    << float(yellow_lane_detection.at<uchar>(i, j))
          //                    *
          //                           (float(j) / float(cropped_image.cols))
          //                    << std::endl;
          //          std::cout << float(yellow_lane_detection.at<uchar>(i, j))
          //                    << std::endl;
        }
        if ((float(yellow_lane_detection.at<uchar>(i, j)) * float(j) /
             float(cropped_image.cols)) > 255 * 0.8) {
          yellow_total += float(yellow_lane_detection.at<uchar>(i, j));
          yellow_centroid += float(yellow_lane_detection.at<uchar>(i, j)) *
                             float(j) / float(cropped_image.cols);
        }
        white_total += float(while_lane_detection.at<uchar>(i, j));
        white_centroid += float(while_lane_detection.at<uchar>(i, j)) *
                          float(j) / float(cropped_image.cols);
      }
    }
    if (white_total > 0)
      white_centroid /= white_total;
    if (yellow_total > 0)
      yellow_centroid /= yellow_total;
    // Fallback value if we can't see yellow
    if (yellow_total == 0)
      yellow_centroid = 1 - white_centroid;
    old_centroid = yellow_centroid;
    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
            .toImageMsg();
    LaneControl::pub.publish(msg2);
    //    std::cout << "White centroid " << yellow_centroid << std::endl;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //        std::cout << duration.count() << std::endl;

    //    control_msg.stamp = msg->stamp;
    bool right = false;
    bool straight = false;
    bool left = false;
    if (yellow_centroid < 0.14)
      left = true;
    else if (yellow_centroid > 0.20)
      right = true;
    else
      straight = true;

    //    std::cout << "Going straigh\n";
    //    if(1-white_centroid > yellow_centroid*1.1)
    //      left = true;
    //    else if(1 - white_centroid < 0.9*yellow_centroid)
    //      right = true;
    //    else
    //      straight = true;

    if (right) {
      control_msg.vel_right = 0.05;
      control_msg.vel_left = 0.6;
    } else if (left) {
      control_msg.vel_right = 0.6;
      control_msg.vel_left = 0.05;
    } else {
      control_msg.vel_right = 0.3;
      control_msg.vel_left = 0.3;
    }
    LaneControl::lane_control_pub.publish(control_msg);
  }
};

void mySigintHandler(int sig) {
  duckietown_msgs::WheelsCmdStamped control_msg;
  control_msg.vel_right = 0.0;
  control_msg.vel_left = 0.0;
  LaneControl::lane_control_pub.publish(control_msg);
  ros::shutdown();
}

apriltag_family_t *LaneControl::tf = tag36h11_create();
apriltag_detector_t *LaneControl::td = apriltag_detector_create();
image_transport::Publisher LaneControl::pub;
image_transport::Publisher LaneControl::april_tag_pub;
ros::Time LaneControl::last_detection;
float LaneControl::old_centroid = 1;
std::vector<int> LaneControl::detected;
int LaneControl::global_t = 0;

ros::Publisher LaneControl::lane_control_pub;
int main(int argc, char **argv) {
  std::cout << "OVERWRITING CONTROL COMMANDS\n";
  ros::init(argc, argv, "image_listener_control");
  ros::NodeHandle nh;
  LaneControl::last_detection = ros::Time::now();
  apriltag_detector_add_family(LaneControl::td, LaneControl::tf);
  image_transport::ImageTransport it(nh);
  LaneControl::lane_control_pub =
      nh.advertise<duckietown_msgs::WheelsCmdStamped>(
          "/control_commands/wheels_cmd", 1);
  image_transport::Subscriber sub = it.subscribe("/csc22940/camera_node/image",
                                                 1, LaneControl::imageCallback);
  LaneControl::pub = it.advertise("/lane_control/image", 1);
  LaneControl::april_tag_pub = it.advertise("/april_tag/image", 1);
  signal(SIGINT, mySigintHandler);
  ros::spin();
}
