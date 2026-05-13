import math
from functools import partial
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import Float64


class Arm_Teleop:
    node: Node

    def __init__(self, teleop_node):
        self.arm_point = Point()
        self.wrist_axis = Float64()
        self.gripper_axis = Float64()
        self.node = teleop_node

        # initialize parameters
        self.node.declare_parameter('arm/button.home', 5)
        self.node.declare_parameter('arm/button.use_arm_mode', 0)
        self.node.declare_parameter('arm/axis.x', 1)
        self.node.declare_parameter('arm/axis.y', 0)
        self.node.declare_parameter('arm/axis.wrist', 2)
        self.node.declare_parameter('arm/axis.gripper', 5)
        self.node.declare_parameter('arm/axis.throttle', 3)
        self.node.declare_parameter('arm/speed_multiplier', 10.0)

        # initialize subscribers, subscribers, timers and service clients
        self.point_publisher = self.node.create_publisher(Point, '/arm_control_relative', 10)
        self.wrist_publisher = self.node.create_publisher(Float64, '/wrist_control_relative', 10)
        self.gripper_publisher = self.node.create_publisher(Float64, '/endeffector_control_relative', 10)
        self.home_client     = self.node.create_client(Trigger, "/home_arm", callback_group=ReentrantCallbackGroup())

        self.node.add_publisher_to_main_timer(self.pub_timer_callback)

    def pub_timer_callback(self):
        """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
        # TODO: obtain arm length values
        maxValue = math.sqrt(150 ** 2 + 260 ** 2)


         # map -1..1 to 0..1
        def scale_js_axis(value):
            return (value + 1.0) / 2

        throttle_scaler = scale_js_axis(self.node.get_axis_value('arm/axis.throttle'))
        arm_speed_scaler = self.node.get_param_val('arm/speed_multiplier').double_value * throttle_scaler
        if self.node.get_button_held('arm/button.use_arm_mode'):

            self.arm_point.x = self.node.get_axis_value('arm/axis.x') * arm_speed_scaler
            self.arm_point.y = - self.node.get_axis_value('arm/axis.y') * arm_speed_scaler
            self.wrist_axis.data = - self.node.get_axis_value('arm/axis.wrist') * arm_speed_scaler * 0.4
        self.gripper_axis.data = - self.node.get_axis_value('arm/axis.gripper') * arm_speed_scaler * 0.5

        self.point_publisher.publish(self.arm_point)
        self.wrist_publisher.publish(self.wrist_axis)
        self.gripper_publisher.publish(self.gripper_axis)
        self.arm_point.x = 0.0
        self.arm_point.y = 0.0
        self.gripper_axis.data = 0.0
        self.wrist_axis.data = 0.0
        if self.node.get_button_pressed('arm/button.home'):
            self.send_home_request()

    def send_home_request(self):
        if not self.home_client.service_is_ready():
            self.node.get_logger().warn('arm home service unavailable, aborting request.')
            return

        future = self.home_client.call_async(Trigger.Request())
        def callback(self, fk_u_python): # neccesary because python has crippled lambdas
            fk_u_python.node.get_logger().info('home arm success' if self.result().success else 'home arm fail')
        future.add_done_callback(partial(callback, fk_u_python=self))

    def _scale(self, num, inMin, inMax, outMin, outMax):
        """scale inputs min/max values, used for mapping joystick data"""
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))
