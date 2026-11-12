import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo

class ReachNode(Node):
    def __init__(self):
        super().__init__('reach_node')

        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info', qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.5)
        self.sync.registerCallback(self.on_synced_frames)

        self.get_logger().info('reach_node started, waiting for synchronized camera frames...')

    def on_synced_frames(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        self.get_logger().info(f'got synced frame: RGB timestamp={rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec}, Depth timestamp={depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec}')


def main():
        rclpy.init()
        node = ReachNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()