import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DroNode(Node):
    def __init__(self):
        super().__init__('dro_node')
        self.get_logger().info('DRO node started')

        # Check parameter for robot model - default to auto-mate arm
        self.robot_model = self.declare_parameter('robot_model', 'xm540arm').value

        # Subscribe to position topic for robot
        self.subscription = self.create_subscription(
            JointState,
            self.robot_model+'/joint_states',
            self.joint_states_callback,
            10)
        
    def joint_states_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.name)


# This class is normally run from the UI, but can be run from the command line
def main(args=None):
    rclpy.init(args=args)
    node = DroNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
