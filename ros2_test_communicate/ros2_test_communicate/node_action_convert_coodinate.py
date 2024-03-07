import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import os
import message_filters
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import estHand_display as ed

bridge = CvBridge()

calibration_path = '/home/fumiya/ros2_ws/src/ros2_test_communicate/calibration'

# mode of depth camera. 0 is AHAT, 1 is LongThrow
mode = 0

class _Mode2_RM_DEPTH_AHAT:
    def __init__(self, uv2xy, extrinsics, scale, alias, undistort_map, intrinsics):
        self.uv2xy         = uv2xy
        self.extrinsics    = extrinsics
        self.scale         = scale
        self.alias         = alias
        self.undistort_map = undistort_map
        self.intrinsics    = intrinsics


class _Mode2_RM_DEPTH_LONGTHROW:
    def __init__(self, uv2xy, extrinsics, scale, undistort_map, intrinsics):
        self.uv2xy         = uv2xy
        self.extrinsics    = extrinsics
        self.scale         = scale
        self.undistort_map = undistort_map
        self.intrinsics    = intrinsics

# RM Depth AHAT Parameters
class Parameters_RM_DEPTH_AHAT:
    WIDTH  = 512
    HEIGHT = 512
    FPS    = 45
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS


# RM Depth Long Throw Parameters
class Parameters_RM_DEPTH_LONGTHROW:
    WIDTH  = 320
    HEIGHT = 288
    FPS    = 5
    PIXELS = WIDTH * HEIGHT
    SHAPE  = (HEIGHT, WIDTH)
    PERIOD = 1 / FPS

class NodeActionConvertCoodinate(Node):
    def __init__(self):
        super().__init__("act_converter")

        self._publisher = self.create_publisher(
            Image,
            '/action_joints',
            10
        )

        qosfile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        sub_i = message_filters.Subscriber(
                self,
                CompressedImage, 
                'act_video_img', 
                qos_profile=qosfile)
        sub_j = message_filters.Subscriber(
                self,
                Image, 
                'act_video_j', 
                qos_profile=qosfile)
        sub_depth = message_filters.Subscriber(
                self,
                Image,
                '/hololens/depth',
                qos_profile=qosfile
        )

        self.check_calibration_directory(calibration_path)
        self.calibration = self.load_calibration_rm(mode, calibration_path)

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
                [sub_i,sub_j,sub_depth], 1000, 0.1)
        self.time_synchronizer.registerCallback(self.callback)

    def callback(self, msg_i: CompressedImage, msg_j: Image, msg_depth: Image):
        #import pdb; pdb.set_trace()
        obj = bridge.compressed_imgmsg_to_cv2(msg_i,'bgr8')
        joint = bridge.imgmsg_to_cv2(msg_j, '32FC3')
        converted_joints = []

        depth_img = bridge.imgmsg_to_cv2(msg_depth, '16UC1')
        adjusted_depth = 1200 - depth_img[depth_img.shape[0]//2, depth_img.shape[1]//2]
        #print("adjust depth : %s" % str(adjusted_depth))

        #video = ed.act_overlay(obj, joint)
        #cv2.namedWindow('act', cv2.WINDOW_NORMAL)
        #cv2.imshow("act", video[0])

        #pdb.set_trace()

        for i in range(msg_j.height):
            converted_joints.append(self.convert_image_to_cameraspace(self.calibration, joint[i, :, :], adjusted_depth))

        converted_joints = np.asarray(converted_joints).astype(np.float32)

        message :Image = bridge.cv2_to_imgmsg(converted_joints, '32FC3')
        message.header.stamp = msg_j.header.stamp
        message.header.frame_id = msg_j.header.frame_id

        self._publisher.publish(message)
        print(converted_joints[0, 0, :])

    def convert_image_to_cameraspace(self, calibration, data, adjusted_depth) -> np.array:
        cameraspace_joint_points = []

        k = [[calibration.intrinsics[0, 0], 0, calibration.intrinsics[2, 0]],[0, calibration.intrinsics[1, 1], calibration.intrinsics[2, 1]], [0, 0, 1]]
        k = np.asarray(k)
        k_inv = np.linalg.inv(k)

        for part in range(len(data)):
            cameraspace_point = []

            pixel_position = np.asarray([data[part, 0], data[part, 1], 1]).T
            m = (data[part, 2] * np.matmul(k_inv, pixel_position)).T
            # if part == 0:
            #     print(m)
            cameraspace_point.append(m[0])
            cameraspace_point.append(m[1])
            cameraspace_point.append(data[part, 2])# - adjusted_depth)

            cameraspace_joint_points.append(cameraspace_point)
        
        return cameraspace_joint_points
    
    def check_calibration_directory(self, path):
        if (not os.path.isdir(path)):
            raise IOError('Calibration path ' + path + ' does not exist')
    
    def load_calibration_rm(self, mode, path):
        if (mode == 0):
            return self.load_calibration_rm_depth_ahat(path)
        if (mode == 1):
            return self.load_calibration_rm_depth_longthrow(path)
        return None
    
    def load_calibration_rm_depth_ahat(self, path):
        lut_shape = Parameters_RM_DEPTH_AHAT.SHAPE + (2,)

        path = os.path.join(path, 'rm_depth_ahat')

        uv2xy                 = np.fromfile(os.path.join(path, 'uv2xy.bin'),                 dtype=np.float32).reshape(lut_shape)
        extrinsics            = np.fromfile(os.path.join(path, 'extrinsics.bin'),            dtype=np.float32).reshape((4, 4))
        scale                 = np.fromfile(os.path.join(path, 'scale.bin'),                 dtype=np.float32)
        alias                 = np.fromfile(os.path.join(path, 'alias.bin'),                 dtype=np.float32)
        undistort_map         = np.fromfile(os.path.join(path, 'undistort_map.bin'),         dtype=np.float32).reshape(lut_shape)
        intrinsics            = np.fromfile(os.path.join(path, 'intrinsics.bin'),            dtype=np.float32).reshape((4, 4))

        return _Mode2_RM_DEPTH_AHAT(uv2xy, extrinsics, scale, alias, undistort_map, intrinsics)

    def load_calibration_rm_depth_longthrow(self, path):
        lut_shape = Parameters_RM_DEPTH_LONGTHROW.SHAPE + (2,)

        path = os.path.join(path, 'rm_depth_longthrow')

        uv2xy                 = np.fromfile(os.path.join(path, 'uv2xy.bin'),                 dtype=np.float32).reshape(lut_shape)
        extrinsics            = np.fromfile(os.path.join(path, 'extrinsics.bin'),            dtype=np.float32).reshape((4, 4))
        scale                 = np.fromfile(os.path.join(path, 'scale.bin'),                 dtype=np.float32)
        undistort_map         = np.fromfile(os.path.join(path, 'undistort_map.bin'),         dtype=np.float32).reshape(lut_shape)
        intrinsics            = np.fromfile(os.path.join(path, 'intrinsics.bin'),            dtype=np.float32).reshape((4, 4))

        return _Mode2_RM_DEPTH_LONGTHROW(uv2xy, extrinsics, scale, undistort_map, intrinsics)

def main(args=None):
    rclpy.init(args=args)
    subscriber = NodeActionConvertCoodinate()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
