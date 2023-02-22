#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub_image = rospy.Subscriber('/output/image_raw/compressed', CompressedImage, self.callback)

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, 1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        print("Color histogram for color detection = ", np.histogram(hsv[:, :, 0]))
        rospy.loginfo("I heard a message")


if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
