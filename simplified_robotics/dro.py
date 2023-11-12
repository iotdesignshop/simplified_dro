
# Application configuration/setup
import os
os.environ['KIVY_NO_ARGS'] = '1'  # Prevent Kivy from parsing command line arguments

# Default to a narrow, but tall window for the DOFs
from kivy import Config
Config.set('graphics', 'width', '360')
Config.set('graphics', 'height', '800')


from kivy.app import App
from kivy.uix.boxlayout import BoxLayout    
from kivy.core.window import Window
from kivy.properties import NumericProperty, StringProperty, ListProperty, ObjectProperty
from random import random
import colorsys
from ament_index_python.packages import get_package_share_directory
from kivy.core.clipboard import Clipboard
from kivy.clock import Clock
import rclpy
from simplified_robotics.dro_node import DroNode
from kivy.lang import Builder
import math
from kivy.uix.popup import Popup
from kivy.uix.label import Label


# Indicator color constants
GREY_COLOR = [0.5, 0.5, 0.5, 1]
GREEN_COLOR = [24/255.0, 180/255.0, 24/255.0, 1]
YELLOW_COLOR = [220/255.0, 220/255.0, 30/255.0, 1]
ORANGE_COLOR = [230/255.0, 120/255.0, 10/255.0, 1]
RED_COLOR = [255/255.0, 0, 0, 1]

# Package paths for accessing resources
kv_file_path = os.path.join(get_package_share_directory('simplified_robotics'), 'ui/dro.kv')


class DOFWidget(BoxLayout):
    dof_name = StringProperty('Unknown DOF')
    dof_value = NumericProperty(0.0)
    dof_percent = NumericProperty(0.0)
    dof_min = NumericProperty(0.0)
    dof_max = NumericProperty(180.0)
    dof_temp_color = ListProperty(GREY_COLOR)
    dof_force_color = ListProperty(GREY_COLOR)
    image_path = StringProperty(os.path.join(get_package_share_directory('simplified_robotics'), 'ui/images'))



    # Convenience functions for setting temp/force
    def set_temp(self, value):
        self.dof_temp_color = self.calculate_gauge_rgb(value)
    
    def set_force(self, value):
        self.dof_force_color = self.calculate_gauge_rgb(value)
        
    
    # Return a color based on a value from 0-1
    def calculate_gauge_rgb(self, value):
        if (value < 0.4):   # Safe zone
            return GREEN_COLOR
        elif (value < 0.6):  # Warning zone
            return YELLOW_COLOR
        elif (value < 0.8):  # Danger zone
            return ORANGE_COLOR
        else:              # Critical zone
            return RED_COLOR
        
    
class DROLayout(BoxLayout):
    
    scrollview_content = ObjectProperty()
    dof_python_value = StringProperty('[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')

    # Handler for Copy to Clipboard button
    def copy_to_clipboard(self):
        Clipboard.copy(self.dof_python_value)

    

class DROApp(App):
    
    node = None     # To store ROS node after initialization

    def build(self):
        
        # Load the kv file
        Builder.load_file(kv_file_path)

        main_layout = DROLayout()

        return main_layout
    
    def on_start(self):
        # Display a popup to tell the user we're waiting for the robot connection
        self.waitpopup = Popup(title='Please Wait', content=Label(text='Waiting for Robot Connection'),
              auto_dismiss=False)
        self.waitpopup.open()

        # Start ROS node
        self.node = DroNode(self.robot_info_ui_callback, self.robot_position_ui_callback, self.robot_temperature_ui_callback)
        
        # Start a timer to check if the robot is ready
        Clock.schedule_interval(self.wait_robot, 1.0)  # Check for robot connection every second

        # Start the regular ROS update loop
        Clock.schedule_interval(self.spin, 1.0 / 4.0)  # Run ROS node at 4 Hz - this prevents overdriving the UI

    def wait_robot(self, dt):
        if (self.node.ready()):
            # Close the popup
            self.waitpopup.dismiss()
            self.waitpopup = None

            # Stop the timer
            return False
        
    def spin(self, dt):
        # Update ROS Node
        rclpy.spin_once(self.node,timeout_sec=0)

        
    def on_stop(self):
        if (self.node):
            self.node.destroy_node()
        rclpy.shutdown()

    def robot_info_ui_callback(self, robot_info):
        # Create an array of DOF widgets
        self.dof_widgets = []

        # Populate the UI with the information from the robot
        for i in range(robot_info.num_joints):
            widget = DOFWidget()
            widget.dof_name = robot_info.joint_names[i]
            widget.dof_min = robot_info.joint_lower_limits[i]*180.0/math.pi
            widget.dof_max = robot_info.joint_upper_limits[i]*180.0/math.pi
            self.root.scrollview_content.add_widget(widget)
            self.dof_widgets.append(widget)

    def robot_position_ui_callback(self, msg):
        robot_info = self.node.robot_info

        # Update the python value string
        python_value = '['

        # Update the UI with the current joint positions
        for i in range(robot_info.num_joints):
            widget = self.dof_widgets[i]
            widget.dof_value = msg.position[i]*180.0/math.pi
            widget.dof_percent = 100.0*(msg.position[i]-robot_info.joint_lower_limits[i])/(robot_info.joint_upper_limits[i]-robot_info.joint_lower_limits[i])
            
            if (len(msg.effort) > 0): # Simulator doesn't return effort
                # Force is normalized to the current limit for each servo
                widget.set_force(abs(msg.effort[i])/self.node.current_limit[i])    

            python_value += "{:.8f}".format(msg.position[i])
            if (i < robot_info.num_joints-1):
                python_value += ', '

        python_value += ']'
        self.root.dof_python_value = python_value

    def robot_temperature_ui_callback(self, msg):
        robot_info = self.node.robot_info

        # Update the UI with the current joint temperatures
        if (len(self.node.temperature_limit) > 0): # Simulator doesn't return temperature limits
            for i in range(robot_info.num_joints):
                widget = self.dof_widgets[i]

                # 30 degrees is kind of normal operating temp
                temp_min = 30.0
                temp_max = self.node.temperature_limit[i]
                temp_range = temp_max - temp_min
                temp = msg[i] - temp_min
                if temp < 0:
                    temp = 0
                widget.set_temp(temp/temp_range)
            


# Entry point for ROS2
def main(args=None):
    rclpy.init(args=args)
    DROApp().run()

if __name__ == '__main__':
    main()

