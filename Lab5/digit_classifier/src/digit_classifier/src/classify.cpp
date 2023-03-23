#include "../include/nn/networks/graph.h"
#include "../include/utils.h"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <ros/ros.h>
using namespace std::chrono;
// April tag C library imports
//  extern "C" {
//#include "apriltag.h"
//#include "tag36h11.h"
//#include "tag25h9.h"
//#include "tag16h5.h"
//#include "tagCircle21h7.h"
//#include "tagCircle49h12.h"
//#include "tagCustom48h12.h"
//#include "tagStandard41h12.h"
//#include "tagStandard52h13.h"
//#include "common/getopt.h"
//  }

class MNISTModel {
public:
  Graph *model;
  MNISTModel() {
    int a, b;
    float c;
    std::string type;

    this->model = new UtilityPropagation(28 * 28, 0, 0.99999);

    std::ifstream infile("/data/c++_code/src/beginner_tutorials/src/model");
    int old_b = -1;
    while (infile >> a >> b >> c >> type) {
      //      std::cout << "adding feature " << a << std::endl;
      if (b != old_b) {
        this->model->list_of_vertices.push_back(
            VertexFactory::get_vertex(type));
      }
      old_b = b;
    }
    //    this->model->print_graph();
    std::ifstream infile2("/data/c++_code/src/beginner_tutorials/src/model");
    while (infile2 >> a >> b >> c >> type) {
      this->model->add_edge(c, a, b, 1e-4);
    }
    std::cout << "Model initialized\n";
  }
};

class DigitHandler {
public:
  //  static apriltag_family_t *tf = NULL;
  static MNISTModel network;

  static image_transport::Publisher pub;
  static image_transport::Publisher pub2;
  DigitHandler() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    auto start = high_resolution_clock::now();
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat cropped_image = img(cv::Range(0, 250), cv::Range(100, 500));

    //    Detect colors
    int Hue_Lower_Value = 95;         // initial hue value(lower)//
    int Hue_Lower_Upper_Value = 106;  // initial hue value(upper)//
    int Saturation_Lower_Value = 80;  // initial saturation(lower)//
    int Saturation_Upper_Value = 185; // initial saturation(upper)//
    int Value_Lower = 75;             // initial value (lower)//
    int Value_Upper = 170;            // initial saturation(upper)//
    cv::imwrite("/data/test.jpeg", cropped_image);
    cv::createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value,
                       255); // track-bar for lower hue//
    cv::createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value,
                       255); // track-bar for lower-upper hue//
    cv::createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value,
                       255); // track-bar for lower saturation//
    cv::createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value,
                       255); // track-bar for higher saturation//
    cv::createTrackbar("Val_Lower", "Adjust", &Value_Lower,
                       255); // track-bar for lower value//
    cv::createTrackbar("Val_Upper", "Adjust", &Value_Upper,
                       255); // track-bar for upper value//

    cv::Mat convert_to_HSV; // declaring a matrix to store converted image//
    cv::Mat blurred_image;
    cv::GaussianBlur(cropped_image, blurred_image, cv::Size(5, 5), 0);
    cv::cvtColor(blurred_image, convert_to_HSV,
                 cv::COLOR_BGR2HSV); // converting BGR image to HSV and storing
                                     // it in convert_to_HSV matrix//

    cv::Mat detection_screen;
    cv::inRange(
        convert_to_HSV,
        cv::Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower),
        cv::Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper),
        detection_screen); // applying track-bar modified value of track-bar//

    //    cv::erode(detection_screen, detection_screen,
    //              cv::getStructuringElement(
    //                  cv::MORPH_ELLIPSE,
    //                  cv::Size(5, 5))); // morphological opening for removing
    //                  small
    //                                    // objects from foreground//
    //    cv::dilate(detection_screen, detection_screen,
    //               cv::getStructuringElement(
    //                   cv::MORPH_ELLIPSE,
    //                   cv::Size(5, 5))); // morphological opening for removing
    //                   small
    //                                     // object from foreground//
    //    cv::dilate(detection_screen, detection_screen,
    //               cv::getStructuringElement(
    //                   cv::MORPH_ELLIPSE,
    //                   cv::Size(5, 5))); // morphological closing for filling
    //                   up
    //                                     // small holes in foreground//
    //    cv::erode(detection_screen, detection_screen,
    //              cv::getStructuringElement(
    //                  cv::MORPH_ELLIPSE,
    //                  cv::Size(5, 5))); // morphological closing for filling
    //                  up
    //                                    // small holes in foreground//

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(detection_screen, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);
    // draw contours on the original image
    //    cv::drawContours(cropped_image, contours, -1, cv::Scalar(0, 255, 0),
    //    2);
    for (int i = 0; i < contours.size(); i++) {
      cv::Rect box = cv::boundingRect(contours[i]);
      if (box.height * box.width > 1500) {
        float ratio;
        if (box.height > box.width) {
          ratio = float(box.height) / float(box.width);
        } else
          ratio = float(box.width) / float(box.height);
        if (ratio < 1.2) {
          //          std::cout << "Box = " << box.x << " " << std::endl;
          cv::Point p1(box.x, box.y);
          cv::Point p2(box.x + box.width, box.y + box.height);

          cv::Mat cropped_image2 =
              cropped_image(cv::Range(box.y, box.y + box.height - 5),
                            cv::Range(box.x, box.x + box.width));

          cv::Mat resized_image;
          cv::resize(cropped_image2, resized_image, cv::Size(28, 28),
                     cv::INTER_LINEAR);

          cv::Mat black_and_white_resize;
          cv::cvtColor(resized_image, black_and_white_resize,
                       cv::COLOR_BGR2GRAY);
          std::vector<float> input_features;

          int max = -100;
          int min = +500;
          for (int j = 0; j < black_and_white_resize.rows; j++) {
            for (int k = 0; k < black_and_white_resize.cols; k++) {
              float val = 255 - float(black_and_white_resize.at<uchar>(j, k));
              if (val < min) {
                min = val;
              }
              if (val > max) {
                max = val;
              }
              input_features.push_back(val);
            }
          }
          for (int j = 0; j < input_features.size(); j++) {
            input_features[j] = (input_features[j] - min) / (max - min);
            if (input_features[j] < 0.25)
              input_features[j] = 0;
            if (input_features[j] > 0.8)
              input_features[j] = 1;

            resized_image.at<cv::Vec3b>(int(j / 28), j % 28)[2] =
                input_features[j] * 255;
            resized_image.at<cv::Vec3b>(int(j / 28), j % 28)[1] =
                input_features[j] * 255;
            resized_image.at<cv::Vec3b>(int(j / 28), j % 28)[0] =
                input_features[j] * 255;
          }
          //          print_vector(input_features);

          DigitHandler::network.model->set_input_values(input_features);
          float pred = DigitHandler::network.model->update_values();
          //          std::cout << "Prediction = " << pred << std::endl;
          //          std::cout << "Total features = " << input_features.size()
          //          << std::endl;

          //          cv::Mat large_image;
          //          cv::resize(resized_image, large_image, cv::Size(400, 400),
          //                     cv::INTER_LINEAR);
          //          cv::Point text_position(100, 100); // Declaring the text
          //          position// float font_size = 2;               // Declaring
          //          the font size// cv::Scalar font_Color(0, 255, 0); //
          //          Declaring the color of the font// int font_weight = 1; //
          //          Declaring the font weight// if (pred != -1)
          //            cv::putText(large_image, std::to_string(int(pred)),
          //            text_position,
          //                        cv::FONT_HERSHEY_COMPLEX, font_size,
          //                        font_Color, font_weight); // Put

          //          sensor_msgs::ImagePtr msg2 =
          //              cv_bridge::CvImage(std_msgs::Header(), "bgr8",
          //              large_image)
          //                  .toImageMsg();
          //          DigitHandler::pub.publish(msg2);
          cv::rectangle(cropped_image, p1, p2, cv::Scalar(255, 0, 0), 2,
                        cv::LINE_8);
        }
      }
    }
    //
    //

    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
            .toImageMsg();
    DigitHandler::pub2.publish(msg2);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << "Time taken by function: " << duration.count()
              << " milliseconds" << std::endl;
  }
};

MNISTModel DigitHandler::network;
image_transport::Publisher DigitHandler::pub;
image_transport::Publisher DigitHandler::pub2;
// apriltag_family_t DigitHandler::tf = tag36h11_create();
int main(int argc, char **argv) {

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(
      "/csc22940/camera_node/image", 1, DigitHandler::imageCallback);
  DigitHandler::pub = it.advertise("/digits/image", 1);
  DigitHandler::pub2 = it.advertise("/digitsdetection/image", 1);
  ros::spin();
}
