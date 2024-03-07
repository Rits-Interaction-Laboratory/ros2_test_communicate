import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo

from cv_bridge import CvBridge

class DepthImagePreviewNode(Node):
    """天井カメラの深度画像を受け取るためのノード"""

    def __init__(self):
        super().__init__('depth_image_preview_node')

        self.bridge = CvBridge()

        self.frame_count = 0

        # fps計測
        self.measurement_count = 10
        self.before_frame = 0
        self.fps = 0
        self.tm = cv2.TickMeter()
        self.tm.start()

        # QoS Settings
        shigure_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        #subscriber
        depth_subscriber = message_filters.Subscriber(
            self, 
            CompressedImage,
            '/rs/aligned_depth_to_color/compressedDepth',
            qos_profile=shigure_qos
        )

        depth_camera_info_subscriber = message_filters.Subscriber(
            self, 
            CameraInfo,
            '/rs/aligned_depth_to_color/cameraInfo', 
            qos_profile=shigure_qos
        )

        self.time_synchronizer = message_filters.TimeSynchronizer(
                [depth_subscriber], 30000)
        self.time_synchronizer.registerCallback(self.callback)

    def callback(self, depth_src:CompressedImage):
        self.frame_count_up()

        if 'PNG' in depth_src.data[:12]:
            depth_header_size = 0
        else:
            depth_header_size = 12
        raw_data = depth_src.data[depth_header_size:]

        buf = np.ndarray(shape=(1, len(raw_data)),
                        dtype=np.uint8, buffer=raw_data)
        depth_img: np.ndarray = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)

        # グレースケール画像化
        depth_img = depth_img.astype(np.float32)
        depth_img = (depth_img/4096)* 256
        depth_img = depth_img.astype(np.uint8)
        
        depth_img = cv2.applyColorMap(depth_img, cv2.COLORMAP_JET)

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

    depth_image_preview_node = DepthImagePreviewNode()

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

