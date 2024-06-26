import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class NodeController(Node):

    def __init__(self):
        super().__init__('node_controller')
        self.start_cli = self.create_client(Trigger, 'start_node')
        self.stop_cli = self.create_client(Trigger, 'stop_node')

        self.subscription = self.create_subscription(
            Bool,
            '/start_nodes',
            self.listener_callback,
            10
        )

        while not self.start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Start service not available, waiting again...')

        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop service not available, waiting again...')

        self.start_req = Trigger.Request()
        self.stop_req = Trigger.Request()


        

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received start command')
            response = self.send_start_request()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().info(f'Failed to start node: {response.message}')
        elif not msg.data:
            self.get_logger().info('Received stop command')
            response = self.send_stop_request()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().info(f'Failed to stop node: {response.message}')


    def send_start_request(self):
        self.future = self.start_cli.call_async(self.start_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stop_request(self):
        self.future = self.stop_cli.call_async(self.stop_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    rclpy.spin(node_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
