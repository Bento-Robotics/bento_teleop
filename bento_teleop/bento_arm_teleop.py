import math
from array import array
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

# TODO: Wait until Joy listener receives stuff before publishing Twist


class Bento_Teleop(Node):
    arm_point = Point()
    joy_point = Point()
    last_joy = Joy()
    throttle = 0.3

    def __init__(self):
        super().__init__('bento_teleop')

        self.arm_point.x = 110.0
        self.arm_point.y = 0.0

        # initialize parameters
        self.declare_parameter('button.return_home', 3)
        self.declare_parameter('robot_namespace', '/bento')
        self.declare_parameter('axis.linear', 1)
        self.declare_parameter('axis.angular', 2)
        self.declare_parameter('axis.throttle', 3)
        self.declare_parameter('publish_rate', 0.02)

        # initialize subscribers, subscribers, timers and service clients
        self.joy_subscription_    = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.point_publisher_     = self.create_publisher(Point, (self.get_param_val('robot_namespace').string_value + '/arm_position' ), 10)
        self.timer                = self.create_timer(self.get_param_val('publish_rate').double_value, self.timer_callback)
        self.home_client          = self.create_client(Trigger, "/home_arm")

    """joystick message listener, processes joystick data"""
    def joy_callback(self, msg):
        if self.last_joy.header.stamp.sec > msg.header.stamp.sec:
            self.get_logger().warn('Joystick message time travel detected. Check your publishers.')

        self.throttle = self.scale(msg.axes[self.get_param_val('axis.throttle').integer_value], -1.0, 1.0, 0.0, 1.0)
        self.arm_point.x += msg.axes[self.get_param_val('axis.linear').integer_value]
        self.arm_point.y += msg.axes[self.get_param_val('axis.angular').integer_value]
 

        """generic button event parser, takes button name, true on press, false elsewise"""
        def button_got_pressed(button_name):
            button_id = self.get_param_val(button_name).integer_value
            return ( msg.buttons[button_id] == 1  and not  self.last_joy.buttons[button_id] == 1 )


        """generic button event parser, takes button name, true while pressed, false elsewise"""
        def button_is_pressed(button_name):
            button_id = self.get_param_val(button_name).integer_value
            return ( msg.buttons[button_id] == 1 )


        """generic button event parser, takes button name, true on release, false elsewise"""
        def button_got_released(button_name):
            button_id = self.get_param_val(button_name).integer_value
            return ( msg.buttons[button_id] == 0  and  ( self.last_joy.buttons[button_id] == 1 if len(self.last_joy.buttons) >= button_id else False) ) # gets triggered before first write to last_joy


        if button_got_pressed('button.return_home'):
           print("stinkt")
           self.arm_point.x = 110.0
           self.arm_point.y = 0.0
           # self.home_client.call_async(Trigger.Request())

        # save the most recent joystick message, so we can compare with it to figure out what moved
        self.last_joy = msg

    """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
    def timer_callback(self):
        maxValue = math.sqrt(150 ** 2 + 260 ** 2)
        self.arm_point.x += self.joy_point.x * 2 * self.throttle
        self.arm_point.y += self.joy_point.y * 2 * self.throttle

        self.arm_point.x = min(110 + maxValue, max(-maxValue, self.arm_point.x))
        self.arm_point.y = min(maxValue, max(-maxValue, self.arm_point.y))

        self.point_publisher_.publish(self.arm_point)


    """scale inputs min/max values, used for mapping joystick data"""
    def scale(self, num, inMin, inMax, outMin, outMax):
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))

    def get_param_val(self, parameter):
        return self.get_parameter(parameter).get_parameter_value()


def main(args=None):
    rclpy.init(args=args)

    try:
        bento_teleop = Bento_Teleop()

        while rclpy.ok():
            rclpy.spin_once(bento_teleop)

    # shut down cleanly
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
