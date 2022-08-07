#!/usr/bin/env python
import roslib
roslib.load_manifest('drone_description')
import rospy
import tf
from sensor_msgs.msg import Imu


def transform(msg):

    br.sendTransform(
        (0.0, 0.0, 0.0),
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        rospy.Time.now(),
        "base_link",
        "world"
    )


if __name__ == '__main__':
    rospy.init_node('world')
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/imu_pub", Imu, transform)
    rospy.spin()
