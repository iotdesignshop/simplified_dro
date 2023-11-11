import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
from interbotix_xs_msgs.msg import ArmJoy


class ControlNode(Node):
    def __init__(self, robot_torque_state_callback = None):
        super().__init__('simplified_control_node')
        self.get_logger().info('Control node started')

        # Check parameter for robot model - default to auto-mate arm
        self.robot_model = self.declare_parameter('robot_model', 'xm540arm').value
        self.get_logger().info('Robot model: %s' % self.robot_model)
        
        # Create service client for managing torque control
        self.torque_client = self.create_client(TorqueEnable, self.robot_model+'/torque_enable')

        # Create publisher for sending ArmJoy messages
        self.joy_pub = self.create_publisher(ArmJoy, 'commands/joy_processed', 10)
        
        # Stash callbacks
        self.robot_torque_state_callback = robot_torque_state_callback
        
    
    def ready(self):
        # Is torque control ready?
        return self.torque_client.service_is_ready()
        

# This class is normally run from the UI, but can be run from the command line
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
