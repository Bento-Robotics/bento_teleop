import math
from array import array
from functools import partial
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray


class Drive_Teleop():
    velocities = Twist()
    rpm_overrides = Float32MultiArray()
    is_enabled: bool = False
    node: Node


    def __init__(self, teleop_node):
        self.node = teleop_node

        # initialize parameters
        self.node.declare_parameter('drive/axis.linear',   1)
        self.node.declare_parameter('drive/axis.angular',  2)
        self.node.declare_parameter('drive/axis.throttle', 3)
        self.node.declare_parameter('drive/button.enable',  2)
        self.node.declare_parameter('drive/button.disable', 3)
        self.node.declare_parameter('drive/maximum.rpm', Parameter.Type.DOUBLE_ARRAY)
        self.node.declare_parameter('drive/maximum.vel.linear', [10.0, 0.0, 0.0])
        self.node.declare_parameter('drive/maximum.vel.angular', [0.0, 0.0, 10.0])

        self.node.declare_parameter('drive/rpm_override_count', 0)
        for i in range(self.node.get_param_val('drive/rpm_override_count').integer_value):
            override_namespace = 'drive/rpm_override_' + str(i)
            self.node.declare_parameter(override_namespace + '.button_forward', Parameter.Type.INTEGER)
            self.node.declare_parameter(override_namespace + '.button_backward', Parameter.Type.INTEGER)
            self.node.declare_parameter(override_namespace + '.speed_multiplier', Parameter.Type.DOUBLE)
            self.node.declare_parameter(override_namespace + '.overrides_motors', Parameter.Type.INTEGER_ARRAY)

        topics = {
            "ctl_twist": "cmd_vel",
            "ctl_enable": "enable",
            "ctl_rpm": "rpmOverride",
            "max_vel": "maximum/velocity",
            "max_rpm": "maximum/rpms",
        }
        # initialize subscribers, subscribers, timers and service clients
        self.maxRPM_subscription_ = self.node.create_subscription(Float32MultiArray, topics["max_rpm"], self.maximum_rpm_callback, 10)
        self.maxVel_subscription_ = self.node.create_subscription(Twist, topics["max_vel"], self.maximum_velocity_callback, 10)
        self.twist_publisher_     = self.node.create_publisher(Twist, topics["ctl_twist"], 10)
        self.rpm_override_publisher_ = self.node.create_publisher(Float32MultiArray, topics["ctl_rpm"], 10)
        self.enable_client        = self.node.create_client(SetBool, topics["ctl_enable"], callback_group=ReentrantCallbackGroup())
        self.enable_spam_timer = self.node.create_timer(2, self.enable_spam_timer_callback)

        # make sure the service exists already, and send a disable for good measure
        while not self.enable_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('enable service not available yet, waiting...')
        self.send_enable_request(False)

        self.node.add_publisher_to_main_timer(self.pub_timer_callback)

    def send_enable_request(self, data=None):
        """sends a boolean request to the service server,
           response arrives later, via the 'future'
           if data is not None:
           because of multithreading you gotta call it real funkily, so just use bento_teleop_send_enable_request(func)"""
        if data is not None: self.is_enabled = data

        if not self.enable_client.service_is_ready():
            self.node.get_logger().warn('enable service became unavailable, aborting request.')
            return

        req = SetBool.Request()
        req.data = bool(self.is_enabled)
        future = self.enable_client.call_async(req)
        def callback(self, fk_u_python): # neccesary because python has crippled lambdas
            if not future.result().success:  fk_u_python.node.get_logger().warn('Failed to enable robot. Check your drive node.' if fk_u_python.is_enabled else 'Failed to disable robot. Check your drive node.')
            elif data is not None: fk_u_python.node.get_logger().info('enabled robot' if fk_u_python.is_enabled else 'disabled robot')
        future.add_done_callback(partial(callback, fk_u_python=self))


    def pub_timer_callback(self):
        """process joystick data and publish stuff"""
        throttle = self._scale(self.node.get_axis_value('drive/axis.throttle'), -1.0, 1.0, 0.0, 1.0)

        # joystick axes -> Twist, but only if not operating arm
        if not self.node.get_button_held('arm/button.use_arm_mode'):
            try:
                max_linear  = self.node.get_param_val('drive/maximum.vel.linear').double_array_value[0]
                max_angular = self.node.get_param_val('drive/maximum.vel.angular').double_array_value[2]
            except IndexError:
                self.node.get_logger().error('Velocity maximums message malformed. Check your publishers.')

            self.velocities.linear.x  = self._scale(self.node.get_axis_value('drive/axis.linear' ) * throttle, -1.0, 1.0, -max_linear,  max_linear)
            self.velocities.angular.z = self._scale(self.node.get_axis_value('drive/axis.angular') * throttle, -1.0, 1.0, -max_angular, max_angular)
        else:
            self.velocities.linear.x  = 0.0
            self.velocities.angular.z = 0.0


        # get joystick buttons, and send enable/disable requests accordingly
        if self.node.get_button_pressed('drive/button.enable'):
            self.send_enable_request(True)
        if self.node.get_button_pressed('drive/button.disable'):
            self.send_enable_request(False)

        try:
            for i in range(self.node.get_param_val('drive/rpm_override_count').integer_value):
                override_namespace = 'drive/rpm_override_' + str(i)
                speed_multiplier = self.node.get_param_val(override_namespace + '.speed_multiplier').double_value
                overrides_motors = self.node.get_param_val(override_namespace + '.overrides_motors').integer_array_value
                if (direction := self.node.get_button_held(override_namespace + '.button_backward')) or self.node.get_button_held(override_namespace + '.button_forward'):
                    for i in overrides_motors:
                        self.rpm_overrides.data[abs(i)] = throttle * speed_multiplier * (-1 if direction else 1) * (-1 if i<0 else 1) * self.node.get_param_val('drive/maximum.rpm').double_array_value[i]
                else:
                    for i in overrides_motors: self.rpm_overrides.data[abs(i)] = 0
        except IndexError:
            self.node.get_logger().error('RPM override config error: config incompatiable with actual data')

        self.twist_publisher_.publish(self.velocities)
        self.rpm_override_publisher_.publish(self.rpm_overrides)

    def maximum_velocity_callback(self, msg):
        """subscriber callback for max velocity"""
        self.node.set_parameters([
                Parameter('drive/maximum.vel.linear',  Parameter.Type.DOUBLE_ARRAY, [ msg.linear.x,  msg.linear.y,  msg.linear.z  ]),
                Parameter('drive/maximum.vel.angular', Parameter.Type.DOUBLE_ARRAY, [ msg.angular.x, msg.angular.y, msg.angular.z ]),
        ])

    def maximum_rpm_callback(self, msg):
        """subscriber callback for max RPMs"""
        if len(msg.layout.dim) != 2:
            self.node.get_logger().warn('Maximum RPM error: data malformed. Expected 2 dimensions: controllers and motors')

        self.node.set_parameters([
                Parameter('drive/maximum.rpm', Parameter.Type.DOUBLE_ARRAY, array('d', msg.data))
        ])
        # copy override data sizes, but retain our set values
        tmp = self.rpm_overrides.data
        msg.data = [math.nan] * len(msg.data)
        self.rpm_overrides = msg
        self.rpm_overrides.data[0:len(tmp)] = tmp

    def enable_spam_timer_callback(self):
        """gently spam enable service for Bento-Box"""
        if self.is_enabled: self.send_enable_request()


    def _scale(self, num: int|float, inMin, inMax, outMin, outMax):
        """scale inputs min/max values, used for mapping joystick data"""
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))
