#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import Transform, TransformStamped, PoseStamped, Vector3, Quaternion

class endeffector_object_camera_tree:
    def __init__(self):

        rospy.init_node('endeffector_object_camera_tf_tree')
        self.camera_parent_frame = "robot_base_frame"
        self.object_parent_frame = "camera_optical_frame"

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        try:
            self.subscriber = rospy.Subscriber('camera/rgb/endeffector_camera_transformation', TransformStamped, self.transform_cb, queue_size=10)
            rospy.loginfo('Subscribe to the endeffector camera transformation pose topic')
        except rospy.ROSException:
            pass

        self.broadcaster = tf.TransformBroadcaster()

    def transform_cb(self, msg):
        rospy.loginfo("receive the transformation sample!")

        # get the transformation between the robot base and robot end effector
        try:
            #Get the transform between the end effector and the robot base
            self.listener.waitForTransform("robot_base", "robot_end_effector",
                                           msg.header.stamp, rospy.Duration(3.0))
            (trans_1, rot_1) = self.listener.lookupTransform("robot_base", "robot_end_effector", msg.header.stamp)
            robot_trans = Transform(Vector3(*trans_1), Quaternion(*rot_1))
            rospy.loginfo('Transformation between end effector and robot base received!')


            self.listener.waitForTransform("camera_rgb_optical_frame", "QR_object",
                                           msg.header.stamp, rospy.Duration(3.0))
            (trans_2, rot_2) = self.listener.lookupTransform("camera_rgb_optical_frame", "QR_object", msg.header.stamp)
            rospy.loginfo('Transformation between QR object and camera rgb optical frame!')
            camera_trans = Transform(Vector3(*trans_2), Quaternion(*rot_2))
            #print('[INFO] Transformation between end effector and robot base received!')

        except tf.Exception as ex:
            rospy.logwarn(str(ex))
            print(str(ex))
            return

        # create the tf tree
        self.broadcaster.sendTransform(
            (robot_trans.translation.x, robot_trans.translation.y, robot_trans.translation.z),
            (robot_trans.rotation.x, robot_trans.rotation.y, robot_trans.rotation.z, robot_trans.rotation.w),
            rospy.Time.now(),
            "robot_endeffector_frame",
            "robot_base_frame")


        self.broadcaster.sendTransform(
            (camera_trans.translation.x, camera_trans.translation.y, camera_trans.translation.z),
            (camera_trans.rotation.x, camera_trans.rotation.y, camera_trans.rotation.z, camera_trans.rotation.w),
            rospy.Time.now(),
            "QR_object_frame",
            "camera_rgb_optical_frame_frame"
            )


        print(msg.transform.translation.y)
        self.broadcaster.sendTransform(
            (msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z),
            (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w),
            rospy.Time.now(),
            "camera_rgb_optical_frame_frame",
            "robot_endeffector_frame")


        rospy.loginfo('Tf tree published')

if __name__ == "__main__":
    tf_tree = endeffector_object_camera_tree()
    rospy.spin()
