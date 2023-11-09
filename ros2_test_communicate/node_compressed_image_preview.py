import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo

from shigure_core.nodes.node_image_preview import ImagePreviewNode

class CompressedImagePreviewNode(ImagePreviewNode):
    def __init__(self):
        super().__init__("compressed_image_preview_node")

        # QoS Settings
        shigure_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        #subscriber
        color_subscriber = message_filters.Subscriber(
            self, 
            CompressedImage,
            "/rs/color/compressed",
            qos_profile=shigure_qos
        )
        depth_camera_info_subscriber = message_filters.Subscriber(
            self, 
            CameraInfo, 
            '/rs/aligned_depth_to_color/cameraInfo', 
            qos_profile=shigure_qos
        )

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [color_subscriber], 1000)
        self.time_synchronizer.registerCallback(self.callback)

    def callback(self, color_img_src: CompressedImage):
        self.frame_count_up()

        color_img: np.ndarray = self.bridge.compressed_imgmsg_to_cv2(color_img_src)
        height, width = color_img.shape[:2]

        img = self.print_fps(color_img)
        cv2.namedWindow('Preview Color', cv2.WINDOW_NORMAL)
        cv2.imshow('Preview Color', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    compressed_image_preview_node = CompressedImagePreviewNode()

    try:
        rclpy.spin(compressed_image_preview_node)

    except KeyboardInterrupt:
        pass

    finally:
        print()
        # 終了処理
        compressed_image_preview_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()