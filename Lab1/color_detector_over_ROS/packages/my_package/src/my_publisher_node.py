#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rospy
from PIL import Image
from duckietown.dtros import DTROS, NodeType
from numpy import asarray
from sensor_msgs.msg import CompressedImage


class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage)
        image = Image.open("/code/catkin_ws/src/ros-duckietown-test/image_demo.jpg")
        self.image_data = asarray(image)

    def run(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.image_data)[1]).tostring()
            # Publish new image
            self.image_pub.publish(msg)
            print("Publishing message to /output/image_raw/compressed")
            rate.sleep()


if __name__ == '__main__':
    print("PATHPATHPATH\n\nPATHPATH\n")
    print(os.getcwd())
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
