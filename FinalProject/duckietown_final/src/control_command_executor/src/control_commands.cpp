#include "duckietown_msgs/WheelsCmdStamped.h"
#include "std_msgs/Bool.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
using namespace std::chrono;

class LaneControl {

public:
  static bool RIGHT;
  static bool LEFT;
  static bool STRAIGHT;
  static duckietown_msgs::WheelsCmdStamped control_msg;
  static ros::Publisher lane_control_pub;
  static int global_t;
  static void
  control_commands_callback(const duckietown_msgs::WheelsCmdStamped &msg) {
    LaneControl::control_msg.vel_right = msg.vel_right;
    LaneControl::control_msg.vel_left = msg.vel_left;
  }
};

void mySigintHandler(int sig) {
  duckietown_msgs::WheelsCmdStamped control_msg;
  control_msg.vel_right = 0.0;
  control_msg.vel_left = 0.0;
  LaneControl::lane_control_pub.publish(control_msg);
  ros::shutdown();
}

duckietown_msgs::WheelsCmdStamped LaneControl::control_msg;
bool LaneControl::STRAIGHT = false;
bool LaneControl::RIGHT = false;
bool LaneControl::LEFT = false;

ros::Publisher LaneControl::lane_control_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "control_commands");
  ros::NodeHandle nh;
  LaneControl::lane_control_pub =
      nh.advertise<duckietown_msgs::WheelsCmdStamped>(
          "/csc22940/wheels_driver_node/wheels_cmd", 1);

  ros::Rate loop_rate(30);

  signal(SIGINT, mySigintHandler);
  ros::Subscriber sub = nh.subscribe("/control_commands/wheels_cmd", 1,
                                    LaneControl::control_commands_callback);

  int counter = 0;
  while (ros::ok()) {
    counter += 1;
    LaneControl::lane_control_pub.publish(LaneControl::control_msg);
    LaneControl::control_msg.vel_right = 0.0;
    LaneControl::control_msg.vel_left = 0.0;
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}
