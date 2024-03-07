import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros as tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

from ros2_test_communicate_srvs.srv import TransformListen

class NodeHL2TransformListener(Node):
    def __init__(self):
        super().__init__('hololens_tf2_listener')

        self.tf_buffer = Buffer(cache_time= tf2.Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #import pdb; pdb.set_trace()
        self.service = self.create_service(
            TransformListen,
            'hololens_tf2_listen_service',
            self.callback
        )

        # Call on_timer function every second
        # self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'launchPos'
        to_frame_rel = 'hololens_unity'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
            print(t)
            self.get_logger().info(
                f'time : {t.header.stamp.sec}')
            self.get_logger().info(
                f'find transform : {t.transform.translation}'
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        finally:
            print("execute finally code")

        #self.publisher.publish(msg)

    def callback(self, request, response):
        print('request: "%s"' % request)

        requested_time = Time(nanoseconds= request.stamp.sec * 1000000000 + request.stamp.nanosec)

        try:        
            response.transform_stamped = self.tf_buffer.lookup_transform(
                request.parent_frame_id,
                request.child_frame_id,
                requested_time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {request.parent_frame_id} to {request.child_frame_id}: \n{ex}')
            response.transform_stamped = TransformStamped()
            response.transform_stamped.child_frame_id = "failed"
        else:
            self.get_logger().info(
                f'time : {requested_time},\n find transform : {response}'
            )
        finally:
            # print("finally")
            return response


def main():
    rclpy.init()
    node = NodeHL2TransformListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()