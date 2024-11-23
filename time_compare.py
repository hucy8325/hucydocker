import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Image  # 修正消息类型的导入路径

class TimestampLister(Node):
    def __init__(self, topic1, topic2):
        super().__init__('timestamp_lister')
        self.topic1 = topic1
        self.topic2 = topic2

        self.timestamps_topic1 = []
        self.timestamps_topic2 = []

        # 订阅 /livox/lidar
        self.subscription1 = self.create_subscription(
            PointCloud2,  # /livox/lidar 的消息类型
            self.topic1,
            self.listener_callback1,
            10
        )

        # 订阅 /camera/depth/image_rect_raw
        self.subscription2 = self.create_subscription(
            Image,  # /camera/depth/image_rect_raw 的消息类型
            self.topic2,
            self.listener_callback2,
            10
        )

    def listener_callback1(self, msg):
        # 提取 PointCloud2 消息中的时间戳
        timestamp = msg.header.stamp
        time_in_ms = timestamp.sec * 1e3 + timestamp.nanosec / 1e6
        self.timestamps_topic1.append(time_in_ms)
        self.get_logger().info(f'Topic: {self.topic1}, Timestamp: {time_in_ms} ms')

    def listener_callback2(self, msg):
        # 提取 Image 消息中的时间戳
        timestamp = msg.header.stamp
        time_in_ms = timestamp.sec * 1e3 + timestamp.nanosec / 1e6
        self.timestamps_topic2.append(time_in_ms)
        self.get_logger().info(f'Topic: {self.topic2}, Timestamp: {time_in_ms} ms')

def main(args=None):
    rclpy.init(args=args)

    topic1 = '/livox/lidar'  # /livox/lidar 话题名
    topic2 = '/camera/depth/image_rect_raw'  # /camera/depth/image_rect_raw 话题名

    timestamp_lister = TimestampLister(topic1, topic2)

    try:
        rclpy.spin(timestamp_lister)
    except KeyboardInterrupt:
        timestamp_lister.get_logger().info('Shutting down...')
    finally:
        # 确保安全关闭
        if rclpy.ok():
            timestamp_lister.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

