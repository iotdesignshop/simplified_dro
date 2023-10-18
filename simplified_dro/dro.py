from kivy.app import App
from kivy.uix.boxlayout import BoxLayout    
from kivy.core.window import Window
from kivy.properties import NumericProperty, StringProperty, ListProperty, ObjectProperty
from random import random
import colorsys
import os
from ament_index_python.packages import get_package_share_directory
from kivy.core.clipboard import Clipboard
from kivy.clock import Clock
import rclpy
from simplified_dro.dro_node import DroNode
from kivy.lang import Builder
import math

# Default to a narrow, but tall window for the DOFs
Window.size = (360, 800)

# Indicator color constants
GREY_COLOR = [0.5, 0.5, 0.5, 1]

# Package paths for accessing resources
kv_file_path = os.path.join(get_package_share_directory('simplified_dro'), 'ui/dro.kv')


class DOFWidget(BoxLayout):
    dof_name = StringProperty('Unknown DOF')
    dof_value = NumericProperty(0.0)
    dof_min = NumericProperty(0.0)
    dof_max = NumericProperty(180.0)
    dof_temp_color = ListProperty(GREY_COLOR)
    dof_force_color = ListProperty(GREY_COLOR)
    image_path = StringProperty(os.path.join(get_package_share_directory('simplified_dro'), 'ui/images'))



    # Convenience functions for setting temp/force
    def set_temp(self, value):
        self.dof_temp_color = self.calculate_gauge_rgb(value)
    
    def set_force(self, value):
        self.dof_force_color = self.calculate_gauge_rgb(value)
        
    
    # Interpolate color between min and max based on a float value
    def calculate_gauge_rgb(self, value):
        RED_RANGE = 0.9
        GREEN_RANGE = 0.9

        # Clamp value to 0-1
        value = min(max(value, 0.0), 1.0)

        # Interpolate color - going from green to red
        hue = (1.0 - value) * 120 / 360  # Interpolate between red and green hue
        saturation = 1.0  # Full saturation
        value = 0.9      # Not eye burning bright
    
        r,g,b =  colorsys.hsv_to_rgb(hue, saturation, value)
        
        return [r,g,b,1.0]
    
    
class MainLayout(BoxLayout):
    
    
    scrollview_content = ObjectProperty()
    dof_python_value = StringProperty('[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')

    # Handler for Copy to Clipboard button
    def copy_to_clipboard(self):
        Clipboard.copy(self.dof_python_value)

    

class DROApp(App):

    def build(self):
        
        # Load the kv file
        Builder.load_file(kv_file_path)

        main_layout = MainLayout()

        # Start ROS node
        self.node = DroNode(self.robot_info_ui_callback, self.robot_position_ui_callback)

        Clock.schedule_interval(self.spin, 1.0 / 10.0)  # Run ROS node at 10 Hz

        return main_layout
    
    def spin(self, dt):
        # Update ROS Node
        rclpy.spin_once(self.node)

    def on_stop(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def robot_info_ui_callback(self, robot_info):
        print('Robot Info: %s' % robot_info)

        for i in range(robot_info.num_joints):
            widget = DOFWidget()
            widget.dof_name = robot_info.joint_names[i]
            widget.dof_min = robot_info.joint_lower_limits[i]*180.0/math.pi
            widget.dof_max = robot_info.joint_upper_limits[i]*180.0/math.pi
            self.root.scrollview_content.add_widget(widget)

    def robot_position_ui_callback(self, msg):
        pass

# Entry point for ROS2
def main(args=None):
    rclpy.init(args=args)
    DROApp().run()

if __name__ == '__main__':
    main()

