
import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped, PoseStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class HandEyeConnector(object):
    def __init__(self):

        # Frame which is rigidly attached to the camera
        # The transform from this frame to the camera optical frame is what
        # we're trying to compute.
        # For the eye-in-hand case, this is the end-effector frame.
        # For the eye-on-base case, this is the world or base frame.
        self.camera_parent_frame_id = rospy.get_param('~camera_parent_frame')

        # Frame which is rigidly attached to the marker
        # The transform from the camera parent frame to the marker parent frame
        # is given by forward kinematics.
        # For the eye-in-hand case, this is the world or base frame.
        # For the eye-on-base case, this is the end-effector frame.
        self.marker_parent_frame_id = rospy.get_param('~marker_parent_frame')

        self.publish_tf = rospy.get_param('~publish_tf')
        self.tf_suffix = rospy.get_param('~tf_suffix')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.interactive = rospy.get_param('~interactive')

        # Compute the camera base to optical transform
        self.xyz_optical_base = rospy.get_param('~xyz_optical_base', [0,0,0])
        self.rpy_optical_base = rospy.get_param('~rpy_optical_base', [0,0,0])
        self.F_optical_base = PyKDL.Frame(
                PyKDL.Rotation.RPY(*self.rpy_optical_base),
                PyKDL.Vector(*self.xyz_optical_base))
        self.F_base_optical = self.F_optical_base.Inverse()

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # rate limiter
        self.rate = rospy.Rate(self.sample_rate)

        # input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()

        # marker subscriber
        self.QR_subscriber = rospy.Subscriber(
                'visp_auto_tracker/object_position',
                PoseStamped,
                self.pose_cb,
                queue_size=1)
        rospy.loginfo('Subscribe to the QR object pose topic!')

        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
                'compute_effector_camera_quick',
                compute_effector_camera_quick)
        rospy.loginfo('Calibration service available!')



    def compute_calibration(self, msg):
        rospy.loginfo("Computing from %g poses..." % len(self.hand_world_samples.transforms) )
        result = None

        # Get the camera optical frame for convenience
        optical_frame_name = "/camera_rgb_optical_frame"  # frame name for the kinect camera

        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: "+str(ex))
            return None

        if self.publish_tf:
            self.broadcaster.sendTransform(
                    (result.effector_camera.translation.x, result.effector_camera.translation.y, result.effector_camera.translation.z),
                    (result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w),
                    rospy.Time.now(),
                    optical_frame_id + self.tf_suffix,
                    self.camera_parent_frame_id)

        rospy.loginfo("Result:\n"+str(result))

        ec = result.effector_camera
        xyz = (ec.translation.x, ec.translation.y, ec.translation.z)
        xyzw = (ec.rotation.x, ec.rotation.y, ec.rotation.z, ec.rotation.w)
        rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        F_optical_world = PyKDL.Frame(PyKDL.Rotation.Quaternion(*xyzw), PyKDL.Vector(*xyz))
        F_base_world = F_optical_world * self.F_base_optical

        bw = tfconv.toMsg(F_base_world)
        xyz = (bw.position.x, bw.position.y, bw.position.z)
        xyzw = (bw.orientation.x, bw.orientation.y, bw.orientation.z, bw.orientation.w)
        rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        rospy.loginfo("Base xyz: ( %f %f %f ) rpy: ( %f %f %f ) xyzw: ( %f %f %f %f )" % (xyz+rpy+xyzw))

        return result

    def pose_cb(self, msg):

        rospy.loginfo("Received QR code sample.")

        # Get the camera optical frame for convenience
        optical_frame_name = "/camera_rgb_optical_frame"  # frame name for the kinect camera

        try:
            #Get the transform between the end effector and the robot base
            self.listener.waitForTransform(self.camera_parent_frame_name, self.marker_parent_frame_name,
                                           msg.header.stamp, rospy.Duration(0.5))
            (trans, rot) = self.listener.lookupTransform(self.camera_parent_frame_name, self.marker_parent_frame_name. msg.header.stamp)
            rospy.loginfo('Transformation between end effector and robot base received!')
            #print('[INFO] Transformation between end effector and robot base received!')
        except tf.Exception as ex:
            rospy.logwarn(str(ex))
            print(str(ex))
            return

        # update data
        self.world_hand_samples.header.frame_id = self.camera_parent_frame_name
        self.world_hand_samples.transforms.append(Transform(Vector3(*trans), Quarternion(*rot)))

        self.camera_marker_samples.header.frame_id = optical_frame_name
        self.camera_marker_samples.transforms.append(Transform(Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                                                               Quarternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)))

        rospy.loginfo('Transformarrays updated!')
        # print('[INFO] Transformarrays updated!')

        if (len(self.world_hand_samples) != len(self.camera_marker_samples)):
            rospy.logerr("Different numbers of world-hand and camera-marker samples.")
            # print('[ERROR] Different numbers of world-hand and camera-marker samples!')
            return

        n_min = 30
        if self.interactive:
            i = raw_input('Hit [enter] to accept the latest sample, or `d` to discard')
            if i == 'd':
                del self.world_hand_samples.transforms[-1]
                del self.camera_marker_samples.transforms[-1]

            elif (len(self.world_hand_samples.transforms) < n_min):
                rospy.logwarn("%d more samples needed..." % (n_min - len(self.world_hand_samples.transforms)))

            else:
                self.compute_calibration(msg)

            raw_input('Hit [enter] to capture the next sample...')

        else:
            self.rate.sleep()
