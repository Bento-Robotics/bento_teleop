import math
from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bento_teleop.arm_interface import Arm_Teleop
from bento_teleop.drive_interface import Drive_Teleop

    
class Bento_Teleop_Node(Node):
    publisher_callbacks: list = []
    joy_now = Joy()
    joy_last = Joy()


    def __init__(self):
        super().__init__('bento_teleop_node')
        self.declare_parameter('publish_rate', 20.0) # milliseconds
        self.joy_subscription_    = self.create_subscription(Joy, (self.get_namespace() + '_opr/joy'), self.joy_callback, 10)
        self.timer = self.create_timer(self.get_param_val('publish_rate').double_value / 1000, self.pub_timer_callback, autostart=False)

    def begin(self):
        """once everything is initialized, begin processing information
           call after initializing all interfaces"""
        for timer in self.timers:
            timer.reset()
        def update_joy_cb(fk_u_python): # neccesary because python has crippled lambdas and annnoying 'self' requirements
            # save the most recent joystick message, so we can compare with it to figure out what moved
            fk_u_python.joy_last = fk_u_python.joy_now
        self.publisher_callbacks.append(partial(update_joy_cb, fk_u_python=self))

    def add_publisher_to_main_timer(self, function):
        self.publisher_callbacks.append(function)

    def pub_timer_callback(self):
        for publish in self.publisher_callbacks:
            publish()

    def joy_callback(self, msg: Joy):
        """joystick message listener, processes joystick data"""

        if self.joy_last.header.stamp.sec > msg.header.stamp.sec:
            self.get_logger().warn('Joystick message time travel detected. Check your publishers.')
            
        # first call, set up array length
        if self.joy_now.axes == [] and self.joy_now.buttons == []:
            self.joy_now.axes = [0.0] * 6 # set common length,
            self.joy_now.buttons = [False] * 12 # data often incorrect on joy_linux first subscription
            self.joy_last = msg
            return

        # store joystick
        self.joy_now = msg
            
    def get_button_pressed(self, button_param: str) -> bool:
        """button event parser, takes button name, true on press, false elsewise"""
        button_id = self.get_param_val(button_param).integer_value
        try:
            return ( self.joy_now.buttons[button_id] == True and self.joy_last.buttons[button_id] == False )
        except IndexError:
            self.get_logger().error('Joystick message/config error: ' + button_param + ' parameter calls for more buttons than in message.')

    def get_button_held(self, button_param: str) -> bool:
        """button event parser, takes button name, true while pressed, false elsewise"""
        button_id = self.get_param_val(button_param).integer_value
        try:
            return ( self.joy_now.buttons[button_id] == True and self.joy_last.buttons[button_id] == True )
        except IndexError:
            self.get_logger().error('Joystick message/config error: ' + button_param + ' parameter calls for more buttons than in message.')

    def get_button_released(self, button_param: str) -> bool:
        """generic button event parser, takes button name, true on release, false elsewise"""
        button_id = self.get_param_val(button_param).integer_value
        try:
            return ( self.joy_now.buttons[button_id] == False and self.joy_last.buttons[button_id] ==  (True if len(self.joy_last.buttons) >= button_id else False) ) # gets triggered before first write to joy_callback.joy_last
        except IndexError:
            self.get_logger().error('Joystick message/config error: ' + button_param + ' parameter calls for more buttons than in message.')

    def get_axis_value(self, axis_param: str) -> int:
        """get the value of a joystick axis from its index's parameter name"""    
        try:
            return self.joy_now.axes[self.get_param_val(axis_param).integer_value]
        except IndexError:
            self.get_logger().error('Joystick message/config error: ' + axis_param + ' parameter calls for more axes than in message.')
            return math.nan
        
    def get_param_val(self, parameter):
        """get the parameter value of parameter xyz. Use with `.<type>_value` -functions"""
        return self.get_parameter(parameter).get_parameter_value()


def main(args=None):
    rclpy.init(args=args)
    bento_teleop = Bento_Teleop_Node()
    arm_teleop = Arm_Teleop(bento_teleop)
    drive_teleop = Drive_Teleop(bento_teleop)
    bento_teleop.begin()
    
    try:
        rclpy.spin(bento_teleop)
    # shut down cleanly
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    main()
