#include "../include/nn/architure_initializer.h"
#include "../include/nn/networks/graph.h"
#include "../include/nn/weight_initializer.h"
#include "../include/nn/weight_optimizer.h"
#include "../include/utils.h"
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
#include <stdio.h>
using namespace std::chrono;
// April tag C library imports
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

// 1e-3 works for training the model
// We can also drive the car from this topic to save computation. No need to
// pre-process image in a separate thread
class MNISTModel {
public:
  Graph *model;
  Optimizer *opti;
  MNISTModel() {
    float step_size = 1e-2;
    this->model = new UtilityPropagation(36 * 30, 0, 0.999);
    //
    if (true) {
      int a, b;
      float c;
      std::string type;
      std::string model_name = "/data/model_small_4.txt";
      std::ifstream infile(model_name);
      int old_b = -1;
      while (infile >> a >> b >> c >> type) {

        if (b != old_b) {
          this->model->list_of_vertices.push_back(
              VertexFactory::get_vertex(type));
        }
        old_b = b;
      }
      //  network->print_graph();
      std::ifstream infile2(model_name);
      while (infile2 >> a >> b >> c >> type) {
        this->model->add_edge(c, a, b, step_size);
      }

      std::cout << "Model loaded\n";
    } else {

      auto network_initializer = ArchitectureInitializer();
      this->model = network_initializer.initialize_sprase_networks(
          this->model, 20000, 40, "sigmoid", step_size);

      WeightInitializer weight_initializer(-0.02, 0.02, 0);
      this->model = weight_initializer.initialize_weights(this->model);
    }
    this->opti = new Adam(step_size, 0, 0.999, 1e-8, this->model);
    std::cout << "Model initialized\n";
  }
};

class DigitHandler {
public:
  static long long int t;
  static std::map<int, int> labels;
  static apriltag_detector_t *td;
  static apriltag_family_t *tf;
  static std::vector<int> detected;
  static MNISTModel network;

  static image_transport::Publisher pub;
  static image_transport::Publisher pub2;
  static ros::Publisher all_done_publisher;
  DigitHandler() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    auto start = high_resolution_clock::now();
    //    if (DigitHandler::detected.size() >= 4) {
    //      std::cout << "All digits founds; should exit\n";
    //      std_msgs::Bool all_detected;
    //      all_detected.data = true;
    //      DigitHandler::all_done_publisher.publish(all_detected);
    //      ros::shutdown();
    //    }

    if (t % 2000 == 0) {

      std::cout << "Total detected = " << DigitHandler::detected.size()
                << std::endl;
      auto g = DigitHandler::network.model->serialize_graph();
      std::ofstream file("/data/model_small_5.txt");
      file << g;
      std::cout << "Checkpointing model\n";
    }
    DigitHandler::t++;
    //    auto start = high_resolution_clock::now();
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat cropped_image = img(cv::Range(0, 250), cv::Range(110, 530));

    cv::Mat blackandwhite;
    cv::cvtColor(cropped_image, blackandwhite, cv::COLOR_BGR2GRAY);

    image_u8_t img_header = {.width = blackandwhite.cols,
                             .height = blackandwhite.rows,
                             .stride = blackandwhite.cols,
                             .buf = blackandwhite.data};
    zarray_t *detections =
        apriltag_detector_detect(DigitHandler::td, &img_header);
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      float dif = det->p[1][1] - det->p[3][1];
      dif = 0;
      auto bottom_left = cv::Point2f(det->p[0][0] - dif * 0.1,
                                     det->p[0][1] + dif * 0.1 - 1.2 * dif);
      auto bottom_right = cv::Point2f(det->p[1][0] + dif * 0.1,
                                      det->p[1][1] + dif * 0.1 - 1.2 * dif);
      auto top_right = cv::Point2f(det->p[2][0] + dif * 0.1,
                                   det->p[2][1] - dif * 0.1 - 1.2 * dif);
      auto top_left = cv::Point2f(det->p[3][0] - dif * 0.1,
                                  det->p[3][1] - dif * 0.1 - 1.2 * dif);

      auto top_left_t = cv::Point2f(0, 36);
      auto top_right_t = cv::Point2f(30, 36);
      auto bottom_right_t = cv::Point2f(30, 66);
      auto bottom_left_t = cv::Point2f(0, 66);

      std::vector<cv::Point2f> roi_corners = {top_left, top_right, bottom_right,
                                              bottom_left};
      std::vector<cv::Point2f> t_corners = {top_left_t, top_right_t,
                                            bottom_right_t, bottom_left_t};

      cv::Mat M = cv::getPerspectiveTransform(roi_corners, t_corners);
      cv::Mat warped_image;
      cv::Size warped_image_size = cv::Size(30, 66);
      cv::warpPerspective(cropped_image, warped_image, M,
                          warped_image_size); // do perspective transformation
      cv::Mat digit = warped_image(cv::Range(0, 36), cv::Range(0, 30));
      cv::Mat digit_b_and_w;
      cv::cvtColor(digit, digit_b_and_w, cv::COLOR_BGR2GRAY);
      std::vector<float> inp_vector;
      float min = 500;
      float max = -100;
      for (int j = 0; j < digit_b_and_w.rows; j++) {
        for (int k = 0; k < digit_b_and_w.cols; k++) {
          float val = 255 - float(digit_b_and_w.at<uchar>(j, k));
          if (val < min) {
            min = val;
          }
          if (val > max) {
            max = val;
          }
        }
      }

      for (int j = 0; j < digit_b_and_w.rows; j++) {
        for (int k = 0; k < digit_b_and_w.cols; k++) {
          digit_b_and_w.at<uchar>(j, k) =
              int(255 * (float((255 - digit_b_and_w.at<uchar>(j, k)) - min) /
                         float(max - min)));
          inp_vector.push_back(float(digit_b_and_w.at<uchar>(j, k)) /
                               float(255));
        }
      }
      DigitHandler::network.model->set_input_values(inp_vector);
      auto pred = network.model->update_values();

//      float target = DigitHandler::labels[int(det->id)];
//
//      network.model->estimate_gradient(target);
//      network.opti->update_weights(network.model);
      cv::Mat digit_back;
      //      std::cout << "Pred = " << pred << std::endl;
      //      print_vector(network.model->prediction_probabilites);
      //      std::cout << "Target = " << target << " "
      //                << "Numbers left " << 10 - DigitHandler::detected.size()
      //                << std::endl;
      cv::cvtColor(digit_b_and_w, digit_back, cv::COLOR_GRAY2BGR);
      if (network.model->prediction_probabilites[int(pred)] > 0.6) {
        if (std::find(DigitHandler::detected.begin(),
                      DigitHandler::detected.end(),
                      int(pred)) == DigitHandler::detected.end()) {
          DigitHandler::detected.push_back(int(pred));
          std::cout << "New element found";
        }
        cv::putText(digit_back, std::to_string(int(pred)), cv::Point(0, 35),
                    cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 255, 0), 2,
                    false);
      }
      sensor_msgs::ImagePtr msg2 =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", digit_back)
              .toImageMsg();
      DigitHandler::pub.publish(msg2);
      cv::line(cropped_image, top_left, top_right, cv::Scalar(0, 0xff, 0), 2);
      cv::line(cropped_image, top_left, bottom_left, cv::Scalar(0, 0, 0xff), 2);
      cv::line(cropped_image, top_right, bottom_right, cv::Scalar(0xff, 0, 0),
               2);
      cv::line(cropped_image, bottom_right, bottom_left, cv::Scalar(0xff, 0, 0),
               2);
    }

    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
            .toImageMsg();
    DigitHandler::pub2.publish(msg2);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //        std::cout << duration.count() << std::endl;
  }
};

MNISTModel DigitHandler::network;
image_transport::Publisher DigitHandler::pub;
image_transport::Publisher DigitHandler::pub2;
std::vector<int> DigitHandler::detected;
ros::Publisher DigitHandler::all_done_publisher;
apriltag_family_t *DigitHandler::tf = tag36h11_create();
apriltag_detector_t *DigitHandler::td = apriltag_detector_create();
std::map<int, int> DigitHandler::labels;
long long int DigitHandler::t = 0;
// apriltag_detector_add_family(DigitHandler::td, DigitHandler::tf);

int main(int argc, char **argv) {

  //  Adding values to map;
  std::vector<int> b{200, 62, 93, 58, 162, 201, 143, 153, 94, 169};
  for (int i = 0; i < b.size(); i++) {
    DigitHandler::labels[b[i]] = i;
  }

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  apriltag_detector_add_family(DigitHandler::td, DigitHandler::tf);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(
      "/csc22940/camera_node/image", 5, DigitHandler::imageCallback);
  DigitHandler::pub = it.advertise("/digits/image", 1);
  DigitHandler::all_done_publisher =
      nh.advertise<std_msgs::Bool>("alldigitsdetected", 1);
  DigitHandler::pub2 = it.advertise("/digitsdetection/image", 1);
  ros::spin();
}
