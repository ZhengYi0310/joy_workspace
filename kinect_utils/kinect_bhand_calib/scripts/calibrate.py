#!/usr/bin/env python

import rospy
from kinect_bhand_calib import BhandKinectConnector
from kinect_bhand_calib import HandEyeConnector

import tf
import tf_conversions.posemath as tfconv
import PyKDL

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped, PoseStamped

def main():
    rospy.init_node('bhand_kinect')



    listener = tf.TransformListener()

    hec = BhandKinectConnector()
    rospy.spin()


if __name__ == '__main__':
    main()
