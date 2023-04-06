#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage, Range
from std_msgs.msg import Float32, String
from dt_apriltags import Detector
from turbojpeg import TurboJPEG
import cv2
import numpy as np
import tf2_ros
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern

import cv2
import rospy
import tf
import numpy as np

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from dt_apriltags import Detector

import tf2_ros
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import CameraInfo, CompressedImage
import tf

STATES = ["STOP", "FOLLOWING", "WAITING", "LANE", "LEFT", "RIGHT", "STRAIGHT"]

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False

RIGHT = [58, 133]
LEFT = [62, 153]
BOTH = [162, 169]

DEBUG_KHURRAM = True
intersection_tags = [153, 162, 58, 62, 133, 169]


class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")
        self.current_state = "LANE"
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
        self.tail_stop_time = 0
        self.last_tail_detection = 0
        self.last_service_message = 0
        self.intersection_begin_time = 0
        self.CENTER = 320
        self.last_x = None
        self.total_steps_to_take = 0

        # dynamic parameter
        self.detection_freq = DTParam(
            "~detection_freq", default=-1, param_type=ParamType.INT, min_value=-1, max_value=30
        )


        # rospy.wait_for_service("/" + self.veh + '/led_emitter_node/set_pattern')
        # self.log("Service detected")
        # self.led_service = rospy.ServiceProxy("/" + self.veh + '/led_emitter_node/set_pattern', ChangePattern)


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

        # self.distance_sub = rospy.Subscriber("/" + self.veh + "/duckiebot_distance_node/distance", Float32,
        #                                      self.distance_response,
        #                                      queue_size=1)

        self.distance_sub = rospy.Subscriber("/" + self.veh + "/front_center_tof_driver_node/range", Range,
                                             self.distance_response,
                                             queue_size=1)

        self.corners_sub = rospy.Subscriber("/" + self.veh + "/duckiebot_detection_node/centers", VehicleCorners,
                                             self.process_center,
                                             queue_size=1)

        self.jpeg = TurboJPEG()

        self.loginfo("Initialized")


        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 200
        self.velocity = 0.25
        self.twist = Twist2DStamped(v=self.velocity, omega=0)
        self.center_x = None
        self.hard_stop = False
        self.center_y = None
        self.P = 0.015
        self.P_tail = 0.025
        self.not_detected = True
        self.D_tail = -0.002
        self.D = -0.002
        self.pending_update = False
        self.last_error = 0
        self.last_time = rospy.get_time()
        self.time_since_hard_stop = 0

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

    def change_color(self, color_name):
        self.current_state = color_name
        self.log("CURRENT STATE = " + color_name)
        self.last_service_message = rospy.get_time()
        # pass

        # p = String()
        # # self.log(str(p))
        # try:
        #     p.data = color_name
        #     val = self.led_service(p)
        # except Exception as e:
        #     self.log(str(e))

    def process_center(self, msg):
        if(msg.detection.data):
            self.last_tail_detection = rospy.get_time()
            # self.log("Setting last x to " + str( msg.corners[3].x))
            self.last_x = msg.corners[3].x
            if (self.current_state != "FOLLOWING"):
                self.current_state = "FOLLOWING"
                self.change_color("FOLLOWING")


    # def distance_response(self, msg):
    #
    #     try:
    #         msg = msg.data
    #         # self.log("Range = " + str(msg))
    #         self.last_tail_detection = rospy.get_time()
    #         if msg < 0.30:
    #             if(self.current_state != "WAITING"):
    #                 self.current_state = "WAITING"
    #                 self.change_color("WAITING")
    #                 self.log("Stopping and setting time to " + str(rospy.get_time()))
    #             self.last_stop_for_safety = rospy.get_time()
    #         elif msg > 0.30:
    #             if(self.current_state != "FOLLOWING"):
    #                 self.change_color("FOLLOWING")
    #                 self.current_state = "FOLLOWING"
    #                 self.log("Moving")
    #
    #
    #     except Exception as e:
    #         self.log("Distance calculation crashed " + str(e))

    def distance_response(self, msg):

        try:
            msg = msg.range
            if msg < 0.10:
                self.hard_stop = True
                self.time_since_hard_stop = rospy.get_time()

        except Exception as e:
            self.log("Distance calculation crashed " + str(e))


    def detect_april_tags(self, detector_id, msg):
        try:
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
            index_i = -1
            for tag in tags:
                if int(tag.tag_id) in intersection_tags:
                    index_i += 1
                    q = _matrix_to_quaternion(tag.pose_R)
                    p = tag.pose_t.T[0]
                    distance = p[2]
                    # self.log("First position: p[0] val: " + str(p[0]))
                    # self.log("Second position: p[1] val: " + str(p[1]))

                    if distance < 0.20 and rospy.get_time() - self.stop_at_time > 4:
                        self.log("Should stop here")
                        if(self.current_state != "STOP"):
                            self.current_state = "STOP"
                            self.change_color("STOP")
                        self.stop_at_time = rospy.get_time()
        except Exception as e:
            self.log("April tag detection crashed reason: " + str(e))

    def callback(self, msg):

        try:
            if(self.current_state == "FOLLOWING" and rospy.get_time() - self.last_tail_detection > 2):
                self.current_state = "LANE"
                self.change_color("LANE")
                # self.last_x = None
            elif (self.hard_stop and rospy.get_time() - self.time_since_hard_stop > 0.2):
                self.hard_stop = False

            elif self.current_state == "STOP" and rospy.get_time() - self.stop_at_time > 0.8:
                self.log("0.8 seconds passed; moving")
                if(rospy.get_time() - self.last_tail_detection < 0.5):
                    self.change_color("FOLLOWING")
                else:
                    self.change_color("LANE")
                    self.current_state = "LANE"


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
            if(self.current_state == "FOLLOWING"):
                self.log("Gets here")
                if(self.last_x is not None):
                    self.proportional = (self.last_x - self.CENTER)*1
                    self.log("Sedding proportional to " + str(self.proportional))
                    self.pending_update = True
                    self.last_x = None
                    self.total_steps_to_take = 4




            elif(self.current_state == "LANE"):
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
        except Exception as e:
            self.log("Callback function crashed " + str(e))

    def drive(self):

        if self.proportional is None:
            self.twist.omega = 0

        else:
            # P Term
            P = -self.proportional * self.P

            # D Term
            d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
            self.last_error = self.proportional
            # self.log("Error = " + str(self.last_error))
            self.last_time = rospy.get_time()
            D = d_error * self.D

            self.twist.v = self.velocity

            self.twist.omega = P + D


            if DEBUG:
                self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)
        # self.log("Sending control command")
        if self.current_state == "STOP" or self.current_state == "WAITING" or self.hard_stop:
            self.twist.v = 0
            self.twist.omega = 0
            self.last_error = 0
            self.last_time = rospy.get_time()
        if not self.pending_update:
            if self.current_state == "FOLLOWING":
                self.log("Stopping because can't see the leader")
                self.twist.v = 0
                self.twist.omega = 0
                self.last_time = rospy.get_time()



        self.vel_pub.publish(self.twist)
        if self.pending_update:
            self.total_steps_to_take -= 1
            if self.total_steps_to_take == 0:
                self.pending_update = False
                # self.last_x = None



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
