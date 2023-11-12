import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
from interbotix_xs_msgs.msg import ArmJoy


class ControlNode(Node):
    def __init__(self):
        super().__init__('simplified_control_node')
        self.get_logger().info('Control node started')

        # Check parameter for robot model - default to auto-mate arm
        self.robot_model = self.declare_parameter('robot_model', 'xm540arm').value
        self.get_logger().info('Robot model: %s' % self.robot_model)
        
        # Create service client for managing torque control - we don't use this directly
        # but it lets us know when the service is available for use via the API for indirect use
        self.torque_client = self.create_client(TorqueEnable, self.robot_model+'/torque_enable')

        # Create publisher for sending ArmJoy messages
        self.joy_pub = self.create_publisher(ArmJoy, self.robot_model+'/commands/joy_processed', 10)
    
    def ready(self):
        # Is torque control ready?
        return self.torque_client.service_is_ready()
    
    def home(self):
        # Send a home message to the joystick controller to stand up the robot
        joy_msg = ArmJoy()
        joy_msg.pose_cmd = ArmJoy.HOME_POSE
        self.joy_pub.publish(joy_msg)

    def sleep(self):
        # Send a sleep message to the joystick controller to lay down the robot
        joy_msg = ArmJoy()
        joy_msg.pose_cmd = ArmJoy.SLEEP_POSE
        self.joy_pub.publish(joy_msg)

    def torque_off(self):
        # Turn off torque control
        joy_msg = ArmJoy()
        joy_msg.torque_cmd = ArmJoy.TORQUE_OFF
        self.joy_pub.publish(joy_msg)

    def torque_on(self):
        # Turn on torque control
        joy_msg = ArmJoy()
        joy_msg.torque_cmd = ArmJoy.TORQUE_ON
        self.joy_pub.publish(joy_msg)

        

# This class is normally run from the UI, but can be run from the command line
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
