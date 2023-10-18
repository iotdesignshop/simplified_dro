import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.srv import RobotInfo

class DroNode(Node):
    def __init__(self, robot_info_ui_callback = None, robot_position_ui_callback = None):
        super().__init__('dro_node')
        self.get_logger().info('DRO node started')

        # Mark as not ready
        self.robot_info = None  

        # Check parameter for robot model - default to auto-mate arm
        self.robot_model = self.declare_parameter('robot_model', 'xm540arm').value
        
        # Create service client to get robot info
        self.robot_info_client = self.create_client(RobotInfo, self.robot_model+'/get_robot_info')
        while not self.robot_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot Info service not available, waiting again...')
        self.robot_info_request = RobotInfo.Request()

        # Call service to get robot info
        self.robot_info_request.cmd_type = 'group'
        self.robot_info_request.name = 'all'    
        self.future = self.robot_info_client.call_async(self.robot_info_request)
        self.future.add_done_callback(self.robot_info_callback)

        # Stash callbacks
        self.robot_info_ui_callback = robot_info_ui_callback
        self.robot_position_ui_callback = robot_position_ui_callback


    def robot_info_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Robot Info: %s' % response)
            self.robot_info = response

        # Create subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.robot_model+'/joint_states',
            self.joint_states_callback,
            10)
        
        # Notify UI
        if self.robot_info_ui_callback:
            self.robot_info_ui_callback(self.robot_info)
        
        
    def joint_states_callback(self, msg):
        # Notify UI
        if self.robot_position_ui_callback:
            self.robot_position_ui_callback(msg)
        

# This class is normally run from the UI, but can be run from the command line
def main(args=None):
    rclpy.init(args=args)
    node = DroNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
