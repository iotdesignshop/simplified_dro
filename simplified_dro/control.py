
# Application configuration/setup
import os
os.environ['KIVY_NO_ARGS'] = '1'  # Prevent Kivy from parsing command line arguments

# Default to a narrow, but tall window for the DOFs
from kivy import Config
Config.set('graphics', 'width', '240')
Config.set('graphics', 'height', '240')


from kivy.app import App
from kivy.uix.boxlayout import BoxLayout    
from kivy.core.window import Window
from kivy.properties import NumericProperty, StringProperty, ListProperty, ObjectProperty
from random import random
from ament_index_python.packages import get_package_share_directory
from kivy.clock import Clock
import rclpy
from simplified_dro.control_node import ControlNode
from simplified_dro.dro_node import DroNode
from kivy.lang import Builder
import math
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, Line
from kivy.metrics import dp



# Package paths for accessing resources
kv_file_path = os.path.join(get_package_share_directory('simplified_dro'), 'ui/dro.kv')

class CountdownButton(Button):
    clock_rate = 0.1
    countdown_time = 1.0
    cd_color = None
    cd_arc = None
    countdown_event = None

    
    # On button press, start countdown
    def on_press(self):
        # If torque is on, countdown to turn it off for safety reasons
        if self.text == 'ON':
            assert (self.countdown_event is None)
            self.countdown = self.countdown_time
            self.countdown_event = Clock.schedule_interval(self.countdown_callback, self.clock_rate)

            # Start drawing the countdown arc
            with self.canvas:
                self.cd_color = Color(1, 1, 0)
                self.cd_arc = Line(circle=(self.center_x, self.center_y, dp(54), 0, 360),width=dp(6))

        else:
            # Turning torque on is instant
            self.text = 'ON'

    def cancel_countdown(self):

        if (self.countdown_event is not None):
            self.countdown_event.cancel()
            self.countdown_event = None

        if (self.cd_arc is not None):
            self.canvas.remove(self.cd_arc)
            self.canvas.remove(self.cd_color)
            self.cd_arc = None
            self.cd_color = None

    # On button release, cancel countdown
    def on_release(self):
        
        if (self.countdown > 0):
            self.text = 'ON'
            self.cancel_countdown()

        else:
            if (self.text == 'ON'):
                # Let main layout know that torque is on
                self.parent.parent.torque_on()
            

    # Callback for countdown
    def countdown_callback(self, dt):
        self.countdown -= self.clock_rate
        countdown_angle = self.countdown / self.countdown_time * 360.0
        self.text = 'HOLD'

        # Update the countdown arc
        if (self.cd_arc is not None):
            self.cd_arc.circle = (self.center_x, self.center_y, dp(54), 0, countdown_angle)
        
        if self.countdown <= 0:
            self.text = 'OFF'
            self.cancel_countdown()
            self.parent.parent.torque_off()
    pass

class ControlLayout(BoxLayout):
    def set_node(self, node):
        self.node = node
    
    def home_pressed(self):
        self.node.home()

    def sleep_pressed(self):
        self.node.sleep()

    def torque_off(self):
        self.node.torque_off()

    def torque_on(self):
        self.node.torque_on()
        


class ControlApp(App):
    
    node = None     # To store ROS node after initialization

    def build(self):
        
        # Load the kv file
        Builder.load_file(kv_file_path)
        self.main_layout = ControlLayout()

        return self.main_layout
    
    def on_start(self):
        # Display a popup to tell the user we're waiting for the robot connection
        self.waitpopup = Popup(title='Please Wait', content=Label(text='Waiting for Robot Connection'),
              auto_dismiss=False)
        self.waitpopup.open()

        # Start ROS node
        self.node = ControlNode()
        self.main_layout.set_node(self.node)
        
        # Start a timer to check if the robot is ready
        Clock.schedule_interval(self.wait_robot, 1.0)  # Check for robot connection every second

        # Start the regular ROS update loop
        Clock.schedule_interval(self.spin, 1.0 / 4.0)  # Run ROS node at 4 Hz - this prevents overdriving the UI

    def wait_robot(self, dt):
        # Once control node is ready, we can close the "wait" popup
        if (self.node.ready()):
            # Close the popup
            self.waitpopup.dismiss()
            self.waitpopup = None

            # Stop the timer
            return False
        
    def spin(self, dt):
        # Update ROS Node
        rclpy.spin_once(self.node,timeout_sec=0)
        pass

        
    def on_stop(self):
        if (self.node):
            self.node.destroy_node()
        rclpy.shutdown()

        
    
# Entry point for ROS2
def main(args=None):
    rclpy.init(args=args)
    ControlApp().run()

if __name__ == '__main__':
    main()

