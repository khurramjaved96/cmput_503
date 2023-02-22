#!/usr/bin/env python3

import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge
import rospy
import numpy as np


UP = [0.1, 0]
DOWN = [-0.1, 0]
LEFT = [0.1, 2]
RIGHT = [0.1, -2]

# class PIDController:
#     def __init__(self, Kp, Ki, Kd):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.error_sum = 0
#         self.last_error = 0
#         self.last_time = rospy.get_time()
#
#     def control(self, error):
#         current_time = rospy.get_time()
#         delta_time = current_time - self.last_time
#         delta_error = error - self.last_error
#         self.error_sum += error * delta_time
#
#         output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Kd * (delta_error / delta_time))
#
#         self.last_error = error
#         self.last_time = current_time
#
#         return output


class LaneFollowingNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)  # NodeType?

        # Set up the publishers and subscribers
        # self.robot_name = rospy.get_namespace().strip("/")


        self.sub_image = rospy.Subscriber("csc22938/camera_node/image/compressed", CompressedImage,
                                          self.process_image)
        self.pub_cmd = rospy.Publisher("csc22938/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

        # Set the initial target v and omega
        self.target_v = 0.1
        self.target_omega = 0.0

        # Set up the PID controllers
        # TODO: tune the controller parameters with trial and error
        # self.v_controller = PIDController(Kp=1, Ki=1, Kd=1)
        # self.omega_controller = PIDController(Kp=1, Ki=1, Kd=1)

        # Some initializations for working with camera images
        self.bridge = CvBridge()

        self.log("Lane Following Node Initialized.")

    def control_wheels(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = self.target_v
        cmd_msg.omega = self.target_omega
        self.pub_cmd.publish(cmd_msg)

    def calculate_error(self, img):
        # TODO: Implement the image processing function to calculate the error in the lane following task
        # TODO: Save errors in a rosbag to analyze later
        img = cv2.resize(img, (16, 16))
        img_hsv = hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        range_color = [23, 28]
        img_hsv = img_hsv[:, :, 0] * (img_hsv[:, :, 0] > range_color[0]) * (img_hsv[:, :, 0] < range_color[1])
        sum_val = np.sum(img_hsv, axis=0) / np.sum(img_hsv)
        first_eight = np.sum(sum_val[0:8])
        last_eight = np.sum(sum_val[8:16])
        error = first_eight - last_eight
        return error
        # v_error = 0
        # omega_error = 0
        # return v_error, omega_error

    def process_image(self, msg):
        # Convert the compressed image to OpenCV format
        img = self.bridge.compressed_imgmsg_to_cv2(msg)

        # Convert the image to grayscale
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Calculate the errors based on the processed image
        # v_error, omega_error = self.calculate_error(img)
        error = self.calculate_error(img)
        if(abs(error) < 0.4):
            self.target_v = UP[0]
            self.target_omega = UP[1]
        elif(error > 0):
            self.target_v = LEFT[0]
            self.target_omega = LEFT[1]
        elif(error < 0):
            self.target_v = RIGHT[0]
            self.target_omega = RIGHT[1]
        # Calculate the target v and omega
        # self.target_v = self.v_controller.control(v_error)
        # self.target_omega = self.omega_controller.control(omega_error)

        # Update the command to the wheels
        self.control_wheels()


if __name__ == '__main__':
    # create the node
    node = LaneFollowingNode(node_name='lane_following_node')

    # keep spinning
    rospy.spin()





