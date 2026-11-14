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

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        u_px = int(round(u))
        v_px = int(round(v))
        u_px = max(0, min(u_px, depth_image.shape[1] - 1))
        v_px = max(0, min(v_px, depth_image.shape[0] - 1))

        depth_value = float(depth_image[v_px, u_px])

        MAX_VALID_DEPTH = 5.0  # meters — generic sanity bound for this scene, see report.md
        
        if not np.isfinite(depth_value) or depth_value <= 0.0 or depth_value > MAX_VALID_DEPTH:
            self.get_logger().warn(f'invalid depth value at pixel (u={u_px}, v={v_px}): {depth_value}')
            return

        Z = depth_value
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        self.get_logger().info(f'3D position of marker in camera frame: (X={X:.3f}, Y={Y:.3f}, Z={Z:.3f})')


def main():
        rclpy.init()
        node = ReachNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()