import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from pythonosc import udp_client

class OSCClientNode(Node):
    
    def __init__(self):
        super().__init__('osc_client_node')

        self.declare_parameter('ip', '192.168.1.255')
        self.declare_parameter('port', 5005)
        self.ip = self.get_parameter('ip').value
        self.port = self.get_parameter('port').value
        self.client = udp_client.SimpleUDPClient(self.ip, self.port)

        self.declare_parameter('topic', 'person_count')
        self.topic = self.get_parameter('topic').value
        self.subscription = self.create_subscription(Int32, self.topic, self.listener_callback, 10)

        self.get_logger().info(f'\n'
            f'\t---OSC Client Node---\n'
            f'\t----------------------\n'
        )

    def listener_callback(self, msg : Int32):
        self.client.send_message("/" + self.topic, msg.data)

def main(args=None):
    rclpy.init(args=args)
    osc_client_node = OSCClientNode()
    rclpy.spin(osc_client_node)
    osc_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()