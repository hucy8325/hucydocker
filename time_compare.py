import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header  # 假设消息包含 Header 信息

class TimestampLister(Node):
    def __init__(self, topic1, topic2):
        super().__init__('timestamp_lister')
        self.topic1 = topic1
        self.topic2 = topic2

        self.timestamps_topic1 = []
        self.timestamps_topic2 = []

        self.subscription1 = self.create_subscription(
            Header,  # 修改为具体消息类型
            self.topic1,
            self.listener_callback1,
            10
        )

        self.subscription2 = self.create_subscription(
            Header,  # 修改为具体消息类型
            self.topic2,
            self.listener_callback2,
            10
        )

    def listener_callback1(self, msg):
        timestamp = Time.from_msg(msg.stamp)
        self.timestamps_topic1.append(timestamp.to_msg().sec * 1e3 + timestamp.to_msg().nanosec / 1e6)
        self.get_logger().info(f'Topic: {self.topic1}, Timestamp: {self.timestamps_topic1[-1]} ms')

    def listener_callback2(self, msg):
        timestamp = Time.from_msg(msg.stamp)
        self.timestamps_topic2.append(timestamp.to_msg().sec * 1e3 + timestamp.to_msg().nanosec / 1e6)
        self.get_logger().info(f'Topic: {self.topic2}, Timestamp: {self.timestamps_topic2[-1]} ms')

def main(args=None):
    rclpy.init(args=args)

    topic1 = '/mid360/output'  # 替换为 my_mid360 的具体输出话题
    topic2 = '/realsense/output'  # 替换为 my_realsense 的具体输出话题

    timestamp_lister = TimestampLister(topic1, topic2)

    try:
        rclpy.spin(timestamp_lister)
    except KeyboardInterrupt:
        timestamp_lister.get_logger().info('Shutting down...')
    finally:
        # 打印收集的所有时间戳
        print(f"Timestamps from {topic1}: {timestamp_lister.timestamps_topic1}")
        print(f"Timestamps from {topic2}: {timestamp_lister.timestamps_topic2}")
        timestamp_lister.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
