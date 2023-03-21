#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <ros/ros.h>

class DigitHandler {
public:
  static image_transport::Publisher pub;
  DigitHandler() {}
  static void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat cropped_image = img(cv::Range(0, 250), cv::Range(100, 500));
    sensor_msgs::ImagePtr msg2 =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image)
            .toImageMsg();
    DigitHandler::pub.publish(msg2);
  }
};

image_transport::Publisher DigitHandler::pub;
int main(int argc, char **argv) {

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(
      "/csc22940/camera_node/image", 1, DigitHandler::imageCallback);
  DigitHandler::pub = it.advertise("/digits/image", 1);
  ros::spin();
  //  cv::destroyWindow("view");
}
