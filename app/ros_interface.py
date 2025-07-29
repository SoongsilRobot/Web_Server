import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ROSInterface:
    def __init__(self):
        rclpy.init()
        self.node = Node("web_ros_node")
        self.publisher = self.node.create_publisher(String, "robot_command", 10)
        self.subscriber = self.node.create_subscription(String, "robot_feedback", self.feedback_callback, 10)
        self.feedback_data = []

        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def send_command(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

    def feedback_callback(self, msg):
        self.feedback_data.append(msg.data)

    def get_latest_feedback(self):
        if self.feedback_data:
            return self.feedback_data.pop(0)
        return None

ros = ROSInterface()