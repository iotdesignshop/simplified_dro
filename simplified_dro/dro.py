from kivy.app import App
from kivy.uix.boxlayout import BoxLayout    
from kivy.core.window import Window
from kivy.properties import NumericProperty, StringProperty, ListProperty, ObjectProperty
from random import random
import colorsys
from kivy.core.clipboard import Clipboard

# Default to a narrow, but tall window for the DOFs
Window.size = (360, 800)

# Indicator color constants
GREY_COLOR = [0.5, 0.5, 0.5, 1]

class DOFWidget(BoxLayout):
    dof_name = StringProperty('Unknown DOF')
    dof_value = NumericProperty(0.0)
    dof_min = NumericProperty(0.0)
    dof_max = NumericProperty(180.0)
    dof_temp_color = ListProperty(GREY_COLOR)
    dof_force_color = ListProperty(GREY_COLOR)


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
        main_layout = MainLayout()

        # Add DOF widgets to scrollview
        for i in range(6):
            main_layout.scrollview_content.add_widget(DOFWidget())

        return main_layout
    

if __name__ == '__main__':
    DROApp().run()

