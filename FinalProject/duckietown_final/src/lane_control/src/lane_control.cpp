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

int STRAIGHT = 56;
int LEFT = 50;
// int RIGHT = 56;
int CROSSING = 163;
int PARK1 = 207;
int PARK2 = 226;
int PARK3 = 228;
int PARK4 = 75;
int PARKBEFORE = 38;
int PARKINGENTRACE = 227;

int STOP_STATE = -100;
int LEFT_TURN_STATE = -101;
int RIGHT_TURN_STATE = -102;
int GO_STRAIGH_STATE = -103;
int NORMAL_NAVIGATION_STATE = -104;

bool DEBUG = false;
bool STOP = false;
class LaneControl {
public:
  static image_transport::Publisher pub;
  static image_transport::Publisher april_tag_pub;
  static ros::Publisher lane_control_pub;
  static float old_centroid;
  static int global_t;
  static int CURRENT_STATE;
  static apriltag_detector_t *td;
  static apriltag_family_t *tf;
  static ros::Time last_detection;
  static ros::Time state_change_time;
  static std::vector<int> detected;
  LaneControl() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    LaneControl::global_t++;
    auto start = high_resolution_clock::now();

    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat cropped_image;
    if (CURRENT_STATE == GO_STRAIGH_STATE)
      cropped_image = img(cv::Range(150, 250), cv::Range(250, 400));
    else if (CURRENT_STATE == LEFT_TURN_STATE)
      cropped_image = img(cv::Range(140, 480), cv::Range(0, 200));
    else if (CURRENT_STATE == RIGHT_TURN_STATE)
      cropped_image = img(cv::Range(270, 480), cv::Range(200, 640));
    else
      cropped_image = img(cv::Range(230, 480), cv::Range(0, 640));

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
      if ((ros::Time::now() - LaneControl::last_detection).toSec() > 2) {
        if (area > 2500) {
          if (int(det->id) == CROSSING)
            LaneControl::change_state(STOP_STATE);
          else if (int(det->id) == LEFT) {
            LaneControl::change_state(LEFT_TURN_STATE);
          } else if (int(det->id) == STRAIGHT) {
            LaneControl::change_state(GO_STRAIGH_STATE);
          }
          LaneControl::last_detection = ros::Time::now();
        }
      }
    }

    //    State specific computation
    if (CURRENT_STATE == STOP_STATE &&
        (ros::Time::now() - state_change_time).toSec() > 1) {
      change_state(NORMAL_NAVIGATION_STATE);
    } else if (CURRENT_STATE == GO_STRAIGH_STATE &&
               (ros::Time::now() - state_change_time).toSec() > 4.0) {
      change_state(NORMAL_NAVIGATION_STATE);
    } else if (CURRENT_STATE == LEFT_TURN_STATE &&
               (ros::Time::now() - state_change_time).toSec() > 2.0) {
      change_state(NORMAL_NAVIGATION_STATE);
    }
    sensor_msgs::ImagePtr msg3 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image_above)
            .toImageMsg();
    LaneControl::april_tag_pub.publish(msg3);

    Mat convert_to_HSV; // declaring a matrix to store converted image//
    cvtColor(cropped_image, convert_to_HSV,
             COLOR_BGR2HSV); // converting BGR image to HSV and storing it in
                             // convert_to_HSV matrix//

    Mat yellow_lane_detection; // declaring matrix for window where object
                               // will be detected//
    inRange(convert_to_HSV, Scalar(22, 66, 100), Scalar(41, 210, 255),
            yellow_lane_detection); // applying track-bar modified value of
                                    // track-bar//

    Mat while_lane_detection;
    if (DEBUG) {

      inRange(convert_to_HSV, Scalar(46, 0, 174), Scalar(171, 82, 255),
              while_lane_detection); // applying track-bar modified value of
                                     // track-bar//
    }

    Mat duck_detection; // declaring matrix for window where object will
                        // be detected//

    inRange(convert_to_HSV, Scalar(13, 114, 149), Scalar(22, 241, 243),
            duck_detection); // applying track-bar modified value of
                             // track-bar//

    duckietown_msgs::WheelsCmdStamped control_msg;
    float yellow_centroid = 0;
    float white_centroid = 0;
    float yellow_total = 0;
    float white_total = 0;
    float duck_total = 0;
    for (int i = 0; i < cropped_image.rows; i++) {
      for (int j = 0; j < cropped_image.cols; j++) {
        duck_total += float(duck_detection.at<uchar>(i, j)) / 255;

        if (DEBUG) {
          cropped_image.at<cv::Vec3b>(i, j)[0] = duck_detection.at<uchar>(i, j);
          cropped_image.at<cv::Vec3b>(i, j)[2] =
              while_lane_detection.at<uchar>(i, j);
        }

        if ((float(yellow_lane_detection.at<uchar>(i, j)) * float(j) /
             float(cropped_image.cols)) < 255 * 0.85) {
          if (DEBUG)
            cropped_image.at<cv::Vec3b>(i, j)[1] =
                yellow_lane_detection.at<uchar>(i, j);
          yellow_total += float(yellow_lane_detection.at<uchar>(i, j));
          yellow_centroid += float(yellow_lane_detection.at<uchar>(i, j)) *
                             float(j) / float(cropped_image.cols);
        }

        float(j) / float(cropped_image.cols);
      }
    }
    if (duck_total > 0) {
      duck_total /= cropped_image.rows * cropped_image.cols;
      if (duck_total > 0.009) {
        change_state(STOP_STATE);
      }
    }

    if (yellow_total > 0)
      yellow_centroid /= yellow_total;

    old_centroid = yellow_centroid;
    if (DEBUG) {
      sensor_msgs::ImagePtr msg2 =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
              .toImageMsg();
      LaneControl::pub.publish(msg2);
    }
    //    std::cout << "White centroid " << yellow_centroid << std::endl;
    if (LaneControl::CURRENT_STATE != STOP_STATE) {
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
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << duration.count() << std::endl;
  }
  static void change_state(int state) {
    std::cout << "Changing state to " << state << std::endl;
    LaneControl::CURRENT_STATE = state;
    state_change_time = ros::Time::now();
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
ros::Time LaneControl::state_change_time;
float LaneControl::old_centroid = 1;
std::vector<int> LaneControl::detected;
int LaneControl::global_t = 0;
int LaneControl::CURRENT_STATE = NORMAL_NAVIGATION_STATE;
ros::Publisher LaneControl::lane_control_pub;

int main(int argc, char **argv) {
  std::cout << "OVERWRITING CONTROL COMMANDS\n";
  ros::init(argc, argv, "image_listener_control");
  ros::NodeHandle nh;
  LaneControl::last_detection = ros::Time::now();
  LaneControl::state_change_time = ros::Time::now();
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
