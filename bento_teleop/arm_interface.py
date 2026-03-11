import math
from functools import partial
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import Float32


class Arm_Teleop:
    node: Node

    def __init__(self, teleop_node):
        self.arm_point = Point()
        self.node = teleop_node
        self.wrist_axis = Float32()
        self.gripper_axis = Float32()
        # initialize parameters
        self.node.declare_parameter('arm/button.home', 5)
        self.node.declare_parameter('arm/button.use_arm_mode', 0)
        self.node.declare_parameter('arm/axis.x', 1)
        self.node.declare_parameter('arm/axis.y', 0)
        self.node.declare_parameter('arm/axis.wrist', 2)
        self.node.declare_parameter('arm/axis.gripper', 5)
        self.node.declare_parameter('arm/axis.throttle', 3)
        self.node.declare_parameter('arm/speed_multiplier', 10.0)
        self.node.declare_parameter('arm/gripper_speed_multiplier', 10.0)


        # initialize subscribers, subscribers, timers and service clients
        self.point_publisher = self.node.create_publisher(Point, '/arm_control_relative', 10)
        self.wrist_publisher = self.node.create_publisher(Float32, '/wrist_control_relative', 10)
        self.gripper_publisher = self.node.create_publisher(Float32, '/endeffector_control_relative', 10)
        self.home_client     = self.node.create_client(Trigger, "/home_arm", callback_group=ReentrantCallbackGroup())

        self.node.add_publisher_to_main_timer(self.pub_timer_callback)

    def pub_timer_callback(self):
        """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
        # TODO: obtain arm length values
        maxValue = math.sqrt(150 ** 2 + 260 ** 2)


        if self.node.get_button_held('arm/button.use_arm_mode'):

            self.arm_point.x += self.node.get_axis_value('arm/axis.x') * self.node.get_param_val('arm/speed_multiplier').integer_value * (self.node.get_axis_value('arm/axis.throttle') + 1.0) * 0.5
            self.arm_point.y += self.node.get_axis_value('arm/axis.y') * self.node.get_param_val('arm/speed_multiplier').integer_value * (self.node.get_axis_value('arm/axis.throttle') + 1.0) * 0.5
            wrist_unscaled = self.node.get_axis_value('arm/axis.wrist') * self.node.get_param_val('arm/speed_multiplier').integer_value * (self.node.get_axis_value('arm/axis.throttle') + 1.0) * 0.5
            gripper_unscaled = self.node.get_axis_value('arm/axis.gripper') * self.node.get_param_val('arm/gripper_speed_multiplier').integer_value * (self.node.get_axis_value('arm/axis.throttle') + 1.0) * 0.5
            self.wrist_axis.data += (wrist_unscaled + 1.0 ) * 0.5
            self.gripper_axis.data += (gripper_unscaled + 1.0 ) * 0.5
        self.point_publisher.publish(self.arm_point)
        self.wrist_publisher.publish(self.wrist_axis)
        self.gripper_publisher.publish(self.gripper_axis)
        self.arm_point.x = 0.0
        self.arm_point.y = 0.0
        self.gripper_axis.data = 0.0
        self.wrist_axis.data = 0.0
        if self.node.get_button_pressed('arm/button.home'):
                # # remove command conflict that was causing jitter
                # self.arm_point.x = 0.0
                # self.arm_point.y = 0.0
                # self.point_publisher.publish(self.arm_point)
                # self.node.call_service(self.send_home_request)
            self.send_home_request()

    def send_home_request(self):
        """because of multithreading you gotta call it real funkily, so just use bento_teleop_node.call_service(func)"""
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
