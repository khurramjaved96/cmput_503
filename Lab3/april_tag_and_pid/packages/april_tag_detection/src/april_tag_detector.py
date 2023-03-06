#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import String
import apriltag
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Transform, Vector3, Quaternion
from dt_apriltags import Detector

TINTERSECTION = [133, 153, 62, 58]
STOP = [169, 162]
UOATAG = [94, 93, 200, 201]
NODETECTION = -1


class ImageDimensionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ImageDimensionNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.time = 0
        self.sub_image = rospy.Subscriber('/csc22938/camera_node/image/compressed', CompressedImage, self.callback,
                                          queue_size=1)
        self.image_pub = rospy.Publisher("/csc22938/camera_node/april_tags/compressed",
                                         CompressedImage, queue_size=1)
        self.detector = apriltag.Detector()
        rospy.wait_for_service("/csc22938/led_emitter_node/set_pattern")
        self.changePattern = rospy.ServiceProxy("/csc22938/led_emitter_node/set_pattern", ChangePattern)
        self.bridge = CvBridge()
        self._tag_pub = rospy.Publisher(
            "~detections",
            AprilTagDetection,
            queue_size=1)


    def _cinfo_cb(self, msg):
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        # find optimal rectified pinhole camera
        with self.profiler("/cb/camera_info/get_optimal_new_camera_matrix"):
            rect_K, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_model.K, self.camera_model.D, (W, H), self.rectify_alpha
            )
            # store new camera parameters
            self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])
        # create rectification map
        with self.profiler("/cb/camera_info/init_undistort_rectify_map"):
            self._mapx, self._mapy = cv2.initUndistortRectifyMap(
                self.camera_model.K, self.camera_model.D, None, rect_K, (W, H), cv2.CV_32FC1
            )
        # once we got the camera info, we can stop the subscriber
        self.loginfo("Camera info message received. Unsubscribing from camera_info topic.")
        # noinspection PyBroadException
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def callback(self, data):

        self.time = (self.time + 1) % 1000
        if (self.time % 10 == 0):
            img = self.bridge.compressed_imgmsg_to_cv2(data)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            result = self.detector.detect(img)
            img = cv2.putText(img, "Apriltag", (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                              1, 255, 2, cv2.LINE_AA)
            max_area = -1000
            max_area_id = NODETECTION
            max_area_tag = None
            print(result)
            for r in result:
                c1 = [int(x - 10) for x in r.corners[0]]
                c2 = [int(x + 10) for x in r.corners[2]]
                area = abs(c1[0] - c2[0]) * abs(c1[1] - c2[1])
                if (area > max_area):
                    max_area = area
                    max_area_id = int(r.tag_id)
                    max_area_tag = r
                cv2.rectangle(img, tuple(c1), tuple(c2), 255, 6)
                center = tuple([int(x) for x in r.center])
                img = cv2.putText(img, str(r.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX,
                                  1, 255, 2, cv2.LINE_AA)

            if max_area_tag is not None:
                tag = max_area_tag
                q = _matrix_to_quaternion(tag.pose_R)
                p = tag.pose_t.T[0]
                # create single tag detection object
                detection = AprilTagDetection(
                    transform=Transform(
                        translation=Vector3(x=p[0], y=p[1], z=p[2]),
                        rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                    ),
                    tag_id=tag.tag_id,
                    tag_family=str(tag.tag_family),
                    hamming=tag.hamming,
                    decision_margin=tag.decision_margin,
                    homography=tag.homography.flatten().astype(np.float32).tolist(),
                    center=tag.center.tolist(),
                    corners=tag.corners.flatten().tolist(),
                    pose_error=tag.pose_err,
                )


            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
            self.image_pub.publish(msg)
            msg = String()

            if (max_area_id in STOP):
                msg.data = "RED"
                self.changePattern(msg)
            elif (max_area_id in UOATAG):
                msg.data = "GREEN"
                self.changePattern(msg)
            elif (max_area_id in TINTERSECTION):
                msg.data = "BLUE"
                self.changePattern(msg)
            elif (max_area_id == NODETECTION):
                if (self.time % 100 == 0):
                    msg.data = "WHITE"
                    self.changePattern(msg)

def _matrix_to_quaternion(r):
    T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)

if __name__ == '__main__':
    # create the node
    node = ImageDimensionNode(node_name='april_tag_node2')
    # keep spinning
    rospy.spin()
