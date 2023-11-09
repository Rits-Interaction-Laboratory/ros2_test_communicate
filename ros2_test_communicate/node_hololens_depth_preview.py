import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

class HololensDepthImagePreviewNode(Node):
    """Hololensから深度画像を受け取るためのノード"""

    def __init__(self):
        super().__init__('hololens_depth_image_preview_node')

        self.bridge = CvBridge()

        # fps計測
        self.frame_count = 0
        self.measurement_count = 10
        self.before_frame = 0
        self.fps = 0
        self.tm = cv2.TickMeter()
        self.tm.start()

        # QoS Settings
        shigure_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        depth_subscriber = self.create_subscription(
            Image,
            '/hololens/depth',
            self.callback,
            qos_profile=shigure_qos
        )

    def callback(self, depth_src:Image):
        # FPSカウントアップ
        self.frame_count_up()

        # Subscribe
        depth_img = self.bridge.imgmsg_to_cv2(depth_src, '16UC1')

        #  グレースケール画像化
        depth_img = depth_img.astype(np.float32)
        depth_img = (depth_img/4096)* 256
        depth_img = depth_img.astype(np.uint8)

        # FPS印字
        depth_img = self.print_fps(depth_img)

        cv2.namedWindow('Preview Depth', cv2.WINDOW_NORMAL)
        cv2.imshow('Preview Depth', depth_img)
        cv2.waitKey(1)

    def frame_count_up(self):
        """
        fpsを計算するためのフレーム計算をします.
        コールバックが呼ばれたときに呼び出す必要があります.

        :return:
        """
        self.frame_count += 1
        # fps計算
        if self.frame_count % self.measurement_count == 0:
            self.tm.stop()
            self.fps = (self.frame_count - self.before_frame) / self.tm.getTimeSec()
            self.before_frame = self.frame_count
            self.tm.reset()
            self.tm.start()

    def print_fps(self, src: np.ndarray) -> np.ndarray:
        """
        fpsを画像に印字します.

        :param src:
        :return:
        """
        img = src

        if src.ndim == 2:
            # 2次元 -> モノクロ画像
            img = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)

        cv2.putText(img, "frame = " + str(self.frame_count), (0, 20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0))
        cv2.putText(img, 'FPS: {:.2f}'.format(self.fps),
                    (0, 40), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0))

        return img

def main(args=None):
    rclpy.init(args=args)

    depth_image_preview_node = HololensDepthImagePreviewNode()

    try:
        rclpy.spin(depth_image_preview_node)

    except KeyboardInterrupt:
        pass

    finally:
        print()
        # 終了処理
        depth_image_preview_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

