import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo

import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.parameter import Parameter

from hw01_sim.arm_client import ArmClient


class ReachNode(Node):
    def __init__(self):
        super().__init__('reach_node', parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.target_camera_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.target_base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)
        self.arm = ArmClient(self)
        self.last_camera_wall_time = time.time()
        self.motion_busy = False

        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info', qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.on_synced_frames)

        self.get_logger().info('reach_node started, waiting for synchronized camera frames...')

    def on_synced_frames(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.last_camera_wall_time = time.time()

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

        point_in_camera = PointStamped()
        point_in_camera.header.stamp = rgb_msg.header.stamp
        point_in_camera.header.frame_id = info_msg.header.frame_id
        point_in_camera.point.x = X
        point_in_camera.point.y = Y
        point_in_camera.point.z = Z
        try:
            transform = self.tf_buffer.lookup_transform('base_link', point_in_camera.header.frame_id, point_in_camera.header.stamp, timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        point_in_base = do_transform_point(point_in_camera, transform)
        self.get_logger().info(f'3D position of marker in base_link frame: (X={point_in_base.point.x:.3f}, Y={point_in_base.point.y:.3f}, Z={point_in_base.point.z:.3f})')
        self.target_camera_pub.publish(point_in_camera)
        self.target_base_pub.publish(point_in_base)

        target_point = PointStamped()
        target_point.header.stamp = point_in_base.header.stamp
        target_point.header.frame_id = point_in_base.header.frame_id
        target_point.point.x = point_in_base.point.x
        target_point.point.y = point_in_base.point.y
        target_point.point.z = point_in_base.point.z + 0.08

        now_sim = self.get_clock().now()
        stamp_time = Time.from_msg(point_in_base.header.stamp)
        observation_age = (now_sim - stamp_time).nanoseconds / 1e9

        if observation_age > 0.5:
            self.get_logger().warn(f'observation too old (age={observation_age:.2f}s), skipping motion')
            return

        camera_data_age = time.time() - self.last_camera_wall_time
        if camera_data_age > 2.0:   
            self.get_logger().warn(f'no fresh camera data for {camera_data_age:.2f}s), skipping motion')
            return

        if self.motion_busy or not self.arm.ready():
            self.get_logger().warn('arm busy with another motion, skipping this detection')     
            return

        self.motion_busy = True
        future = self.arm.move_to_point(target_point)
        future.add_done_callback(self.on_motion_done)

    def on_motion_done(self, future):
        self.motion_busy = False
        result = future.result()
        if result.success:
            self.get_logger().info(f'arm motion completed successfully: {result.message}')
        else:
            self.get_logger().error(f'arm motion failed: {result.message}')

def main():
        rclpy.init()
        node = ReachNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.arm.cancel()
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__':
        main()