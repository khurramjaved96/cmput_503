#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# two almost similar sources I reference for this code:
# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29


PI = 3.1415
apriltag_list = [200, 201, 94, 93, 153, 133, 58, 62, 169, 162]
apriltag_name_list = ['200', '201', '94', '93',
                      '153', '133', '58', '62', '169', '162']
x_list = [0.17, 1.65, 1.65, 0.17, 1.75, 1.253, 0.574, 0.075, 0.574, 1.253]
y_list = [0.17, 0.17, 2.84, 2.84, 1.252, 1.755, 1.259, 1.755, 1.755, 1.253]
yaw = [PI/2 + PI/4, PI/2 + 3*PI/4, -PI/4, PI/2 - PI/4, 0, PI, 0, PI, PI/2, -PI/2 ]
pitch = [1.57]





if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = StaticTransformBroadcaster()
    i = 0
    for i_april in apriltag_list:

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = apriltag_name_list[i]

        t.transform.translation.x = float(x_list[i])
        t.transform.translation.y = float(y_list[i])
        t.transform.translation.z = float(0)

        quat = tf.transformations.quaternion_from_euler( #TODO: search more about it
            float(-PI/2), float(0), yaw[i])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        broadcaster.sendTransform(t)
        # self.log(f"sent info for apriltag {apriltag_name_list[i]}")
        rospy.sleep(0.2)
        i+=1
    # rospy.spin()


