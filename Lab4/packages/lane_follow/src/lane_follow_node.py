#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32
from dt_apriltags import Detector
from turbojpeg import TurboJPEG
import cv2
import numpy as np
import tf2_ros
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped


import cv2
import rospy
import tf
import numpy as np

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from dt_apriltags import Detector
from custom_msgs.srv import ODO

import tf2_ros
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform
from tf2_ros import TransformBroadcaster, Buffer
from tf import transformations as tr
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False

intersection_tags = [153, 162, 58, 62, 133, 169]

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")

        # get static parameters
        self.family = rospy.get_param("~family", "tag36h11")
        self.ndetectors = rospy.get_param("~ndetectors", 1)
        self.nthreads = rospy.get_param("~nthreads", 1)
        self.quad_decimate = rospy.get_param("~quad_decimate", 1.0)
        self.quad_sigma = rospy.get_param("~quad_sigma", 0.0)
        self.refine_edges = rospy.get_param("~refine_edges", 1)
        self.decode_sharpening = rospy.get_param("~decode_sharpening", 0.25)
        self.tag_size = rospy.get_param("~tag_size", 0.065)
        self.rectify_alpha = rospy.get_param("~rectify_alpha", 0.0)
        self.buffer = tf2_ros.Buffer()
        self.stop_at_time = 0


        self.flag = False
        self.stop_flag = False
        # dynamic parameter
        self.detection_freq = DTParam(
            "~detection_freq", default=-1, param_type=ParamType.INT, min_value=-1, max_value=30
        )

        # create a CV bridge object
        self._jpeg = TurboJPEG()

        self._workers = ThreadPoolExecutor(self.ndetectors)
        self._tasks = [None] * self.ndetectors

        # Publishers & Subscribers
        self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="20MB")
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)

        self._cinfo_sub = rospy.Subscriber("/" + self.veh + "/camera_node/camera_info", CameraInfo, self._cinfo_cb,
                                           queue_size=1)

        self.jpeg = TurboJPEG()

        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.4
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()

        # self._detection_reminder = DTReminder(frequency=self.detection_freq.value)
        # camera info
        self._camera_parameters = None
        self._mapx, self._mapy = None, None
        # create detector object
        self._detectors = [
            Detector(
                families=self.family,
                nthreads=self.nthreads,
                quad_decimate=self.quad_decimate,
                quad_sigma=self.quad_sigma,
                refine_edges=self.refine_edges,
                decode_sharpening=self.decode_sharpening,
            )
            for _ in range(self.ndetectors)
        ]


        # Shutdown hook
        rospy.on_shutdown(self.hook)

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



    def detect_april_tags(self, detector_id, msg):
        # self.log("Trying to detect apriltag")
        # turn image message into grayscale image
        with self.profiler("/cb/image/decode"):
            img = self._jpeg.decode(msg.data, pixel_format=TJPF_GRAY)
            # self.log("Image decoded")
        # run input image through the rectification map
        with self.profiler("/cb/image/rectify"):
            img = cv2.remap(img, self._mapx, self._mapy, cv2.INTER_NEAREST)
            # self.log("Image rectified")
        # detect tags
        with self.profiler("/cb/image/detection"):
            tags = self._detectors[detector_id].detect(img, True, self._camera_parameters, self.tag_size)
        # pack detections into a message
        tags_msg = AprilTagDetectionArray()
        tags_msg.header.stamp = rospy.Time.now()
        tags_msg.header.frame_id = msg.header.frame_id
        z_min = 100000
        z_min_id = -1
        index_i = -1
        detec = None
        tran = None
        # print(tags)
        if rospy.get_time() - self.stop_at_time > 2:
            self.stop_flag = False
            # self.log("Running the bot again")
        for tag in tags:
            # self.log("At-least one tag detected")
            # print("SADASDS")
            # print(Vector3(x=p[0], y=p[1], z=p[2]))
            # self.log(str(tag.tag_id))

            index_i += 1
            # turn rotation matrix into quaternion
            q = _matrix_to_quaternion(tag.pose_R)
            p = tag.pose_t.T[0]
            distance = p[2]

            if distance < 0.6 and distance > 0.2 and self.flag == False:
                self.log("Setting flag to True")
                self.flag = True
            if distance < 0.13 and self.flag:
                self.log("Should stop here")
                self.stop_at_time = rospy.get_time()
                self.stop_flag = True
                self.flag = False


            # self.log("Distance = " + str(p[2]))
            # if (p[2] < z_min or True):
            #     z_min = p[2]
            #     z_min_id = index_i
            #     detec = detection
            #     tran = Transform(
            #         translation=Vector3(x=p[0], y=p[1], z=p[2]),
            #         rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            #     )

            # tags_msg.detections.append(detection)
        if(z_min != 100000):
            self.log("Min index = " + str(z_min))
        # self.log("Min tag ID = " + str(tags[z_min_id].tag_id))
            # print("Message stamp", msg.header.stamp)



        # if detec is not None:
            # publish detections
            # self.log("April tag detected")
            # self._tag_pub.publish(detec)

        # if self._img_pub.anybody_listening() and not self._renderer_busy:
        #     self._renderer_busy = True
        #     Thread(target=self._render_detections, args=(msg, img, tags)).start()

    def callback(self, msg):

        # # make sure we have received camera info
        # if self._camera_parameters is None:
        #     return
        # # make sure we have a rectification map available
        # if self._mapx is None or self._mapy is None:
        #     return

        # self.log("Recieving imagel")
        # Code to detect aprilags
        # make sure we have received camera info
        if self._camera_parameters is None:
            return
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        # make sure somebody wants this
        # make sure this is a good time to detect (always keep this as last check)
        # if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
        #     return
        # make sure we are still running
        if self.is_shutdown:
            return
        # ---
        # find the first available worker (if any)
        for i in range(self.ndetectors):
            if self._tasks[i] is None or self._tasks[i].done():
                # submit this image to the pool
                self._tasks[i] = self._workers.submit(self.detect_april_tags, i, msg)
                break

        # self.log("Done with apriltag detection")
        #         Done detecting apriltags
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.proportional = cx - int(crop_width / 2) + self.offset
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = None

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def drive(self):
        if self.proportional is None:
            self.twist.omega = 0
        else:
            # P Term
            P = -self.proportional * self.P

            # D Term
            d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
            self.last_error = self.proportional
            self.last_time = rospy.get_time()
            D = d_error * self.D

            self.twist.v = self.velocity
            if self.stop_flag:
                self.twist.v = 0
            self.twist.omega = P + D
            if DEBUG:
                self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

        self.vel_pub.publish(self.twist)

    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)

def _matrix_to_quaternion(r):
    T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)

if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()