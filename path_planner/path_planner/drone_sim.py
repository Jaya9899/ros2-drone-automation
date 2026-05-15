import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DroneSim(Node):
    def __init__(self):
        super().__init__('drone_sim')

        self.sub = self.create_subscription(
            String,
            '/drone_command',
            self.callback,
            10
        )

    def callback(self, msg):
        cmd = msg.data

        if cmd == "takeoff":
            self.get_logger().info("Taking off...")

        elif cmd == "fly_corridor_1":
            self.get_logger().info("Flying corridor 1...")

        elif cmd == "scan_qr":
            self.get_logger().info("Scanning QR...")

def main(args=None):
    rclpy.init(args=args)
    node = DroneSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()