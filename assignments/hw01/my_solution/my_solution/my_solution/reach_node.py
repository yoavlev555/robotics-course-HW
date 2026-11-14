import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ReachNode(Node):
    def __init__(self):
        super().__init__('reach_node')
        self.bridge = CvBridge()

        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info', qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.on_synced_frames)

        self.get_logger().info('reach_node started, waiting for synchronized camera frames...')

    def on_synced_frames(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'failed to convert RGB image: {e}')
            return
        
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            self.get_logger().warn('no red marker pixels detected in this frame')
            return

        u = float(np.mean(xs))
        v = float(np.mean(ys))

        self.get_logger().info(f'marker detected at pixel (u={u:.1f}, v={v:.1f})')


def main():
        rclpy.init()
        node = ReachNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()