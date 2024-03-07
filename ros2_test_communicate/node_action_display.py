import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import sys
import os
import message_filters
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import estHand_display as ed

bridge = CvBridge()

class MySubscribe(Node):
    def __init__(self):
        super().__init__("my_actsub")
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
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
                [sub_i, sub_j], 1000, 0.1)
        self.time_synchronizer.registerCallback(self.callback)

    def callback(self, msg_i: CompressedImage, msg_j: Image):
        #import pdb; pdb.set_trace()
        obj = bridge.compressed_imgmsg_to_cv2(msg_i,'bgr8')
        joint = bridge.imgmsg_to_cv2(msg_j, '32FC3')
        video = ed.act_overlay(obj, joint)
        #video = np.array(msg.data).reshape(7,150,150,3)
        for f in range(len(video)):
            cv2.namedWindow('act', cv2.WINDOW_NORMAL)
            cv2.imshow("act", video[f])
            cv2.waitKey(100)#"""
        self.get_logger().info("recieve")

def main(args=None):
    rclpy.init(args=args)
    subscriber = MySubscribe()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
