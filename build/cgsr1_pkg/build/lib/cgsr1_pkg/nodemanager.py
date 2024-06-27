import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess

class NodeManager(Node):

    def __init__(self):
        super().__init__('node_manager')
        self.start_srv = self.create_service(Trigger, 'start_node', self.start_node_callback)
        self.stop_srv = self.create_service(Trigger, 'stop_node', self.stop_node_callback)
        self.node_process_computation = None
        self.node_process_automation = None
        self.node_process_basecontrol = None
        self.node_process_brushtest = None
        self.node_process_cameraapriltag = None
        self.node_process_imucontrol = None
        self.node_process_linus_automation = None
        self.node_process_winchcontrol = None

    def start_node_callback(self, request, response):
        if self.node_process_computation is None:
            self.node_process_computation = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'computation'])
        if self.node_process_automation is None:
            self.node_process_automation = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'automation'])
        if self.node_process_basecontrol is None:
            self.node_process_basecontrol = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'basecontrol'])
        if self.node_process_brushtest is None:
            self.node_process_brushtest = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'brushtest'])
        if self.node_process_cameraapriltag is None:
            self.node_process_cameraapriltag = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'cameraapriltag'])
        if self.node_process_imucontrol is None:
            self.node_process_imucontrol = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'imucontrol'])
        if self.node_process_linus_automation is None:
            self.node_process_linus_automation = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'linus_automation'])
        if self.node_process_winchcontrol is None:
            self.node_process_winchcontrol = subprocess.Popen(['ros2', 'run', 'cgsr1_pkg', 'winchcontrol'])
        response.success = True
        response.message = "All nodes started successfully"
        return response

    def stop_node_callback(self, request, response):
        if self.node_process_computation is not None:
            self.node_process_computation.terminate()
            self.node_process_computation.wait()
            self.node_process_computation = None
        if self.node_process_automation is not None:
            self.node_process_automation.terminate()
            self.node_process_automation.wait()
            self.node_process_automation = None
        if self.node_process_basecontrol is not None:
            self.node_process_basecontrol.terminate()
            self.node_process_basecontrol.wait()
            self.node_process_basecontrol = None
        if self.node_process_brushtest is not None:
            self.node_process_brushtest.terminate()
            self.node_process_brushtest.wait()
            self.node_process_brushtest = None
        if self.node_process_cameraapriltag is not None:
            self.node_process_cameraapriltag.terminate()
            self.node_process_cameraapriltag.wait()
            self.node_process_cameraapriltag = None
        if self.node_process_imucontrol is not None:
            self.node_process_imucontrol.terminate()
            self.node_process_imucontrol.wait()
            self.node_process_imucontrol = None
        if self.node_process_linus_automation is not None:
            self.node_process_linus_automation.terminate()
            self.node_process_linus_automation.wait()
            self.node_process_linus_automation = None
        if self.node_process_winchcontrol is not None:
            self.node_process_winchcontrol.terminate()
            self.node_process_winchcontrol.wait()
            self.node_process_winchcontrol = None
        response.success = True
        response.message = "All nodes stopped successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    node_manager = NodeManager()
    rclpy.spin(node_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
