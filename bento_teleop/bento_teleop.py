import math
from array import array
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray

# TODO: Wait until Joy listener receives stuff before publishing Twist


class Bento_Teleop(Node):
    velocities = Twist()
    last_joy = Joy()
    rpm_overrides = Float32MultiArray()

    def __init__(self):
        super().__init__('bento_teleop')

        # initialize parameters
        self.declare_parameter('axis.linear',   1)
        self.declare_parameter('axis.angular',  2)
        self.declare_parameter('axis.throttle', 3)
        self.declare_parameter('button.enable',  2)
        self.declare_parameter('button.disable', 3)
        self.declare_parameter('publish_rate', 0.02)  # seconds
        self.declare_parameter('robot_namespace', Parameter.Type.STRING)
        self.declare_parameter('enable_service_name', '/enable')
        self.declare_parameter('maximum_rpm_topic', '/maximum/rpms')
        self.declare_parameter('maximum_vel_topic', '/maximum/velocity')
        self.declare_parameter('maximum.rpm', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('maximum.vel.linear', [10.0, 0.0, 0.0])
        self.declare_parameter('maximum.vel.angular', [0.0, 0.0, 10.0])

        self.declare_parameter('rpm_override_count', 0)
        for i in range(self.get_param_val('rpm_override_count').integer_value):
            override_namespace = 'rpm_override_' + str(i)
            self.declare_parameter(override_namespace + '.button_forward', Parameter.Type.INTEGER)
            self.declare_parameter(override_namespace + '.button_backward', Parameter.Type.INTEGER)
            self.declare_parameter(override_namespace + '.speed_multiplier', Parameter.Type.DOUBLE)
            self.declare_parameter(override_namespace + '.overrides_motors', Parameter.Type.INTEGER_ARRAY)

        # wait until 'robot_namespace' is set
        while rclpy.ok():
            try: self.get_parameter('robot_namespace')
            except rclpy.exceptions.ParameterUninitializedException:
                self.get_logger().warn('\'robot_namespace\' parameter unset, waiting for value...', once=True)
                rclpy.spin_once(self)
            else: break

        # initialize subscribers, subscribers, timers and service clients
        self.joy_subscription_    = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.maxRPM_subscription_ = self.create_subscription(Float32MultiArray, ( self.get_param_val('robot_namespace').string_value + self.get_param_val('maximum_rpm_topic').string_value ), self.maximum_rpm_callback, 10)
        self.maxVel_subscription_ = self.create_subscription(Twist, ( self.get_param_val('robot_namespace').string_value + self.get_param_val('maximum_vel_topic').string_value ), self.maximum_velocity_callback, 10)
        self.twist_publisher_     = self.create_publisher(Twist, ( self.get_param_val('robot_namespace').string_value + '/cmd_vel' ), 10)
        self.rpm_override_publisher_ = self.create_publisher(Float32MultiArray, ( self.get_param_val('robot_namespace').string_value + '/rpmOverride' ), 10)
        self.timer                = self.create_timer(self.get_param_val('publish_rate').double_value, self.timer_callback)
        self.enable_client        = self.create_client(SetBool, ( self.get_param_val('robot_namespace').string_value + self.get_param_val('enable_service_name').string_value ))

        # make sure the service exists already, and send a disable for good measure
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('enable service not available yet, waiting...', once=True)
        self.send_enable_request(False)


    """sends a boolean request to the service server,
       response arrives later, via the 'future'"""
    def send_enable_request(self, data):
        if not self.enable_client.service_is_ready():
            self.get_logger().warn('enable service became unavailable, aborting request.')
            return

        enable_request = SetBool.Request()
        enable_request.data = data
        self.future = self.enable_client.call_async(enable_request)


    """joystick message listener, processes joystick data"""
    def joy_callback(self, msg):
        if self.last_joy.header.stamp.sec > msg.header.stamp.sec:
            self.get_logger().warn('Joystick message time travel detected. Check your publishers.')

        throttle = self.scale(msg.axes[self.get_param_val('axis.throttle').integer_value], -1.0, 1.0, 0.0, 1.0)

        # get joystick axes, modify linear and angular with throttle
        if self.last_joy.axes != msg.axes:
            try:
                max_linear  = self.get_param_val('maximum.vel.linear').double_array_value[0]
                max_angular = self.get_param_val('maximum.vel.angular').double_array_value[2]
            except IndexError:
                self.get_logger().error('Velocity maximums message malformed. Check your publishers.')

            try:
                self.velocities.linear.x  = self.scale(msg.axes[self.get_param_val('axis.linear').integer_value] * throttle, -1.0, 1.0, -max_linear, max_linear)
                self.velocities.angular.z = self.scale(msg.axes[self.get_param_val('axis.angular').integer_value] * throttle, -1.0, 1.0, -max_angular, max_angular)
            except IndexError:
                self.get_logger().error('Joystick message/config error: parameters call for more axes than in message.')


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


        # get joystick buttons, and send enable/disable requests accordingly
        if self.last_joy.buttons != msg.buttons:
            try:
                if button_got_pressed('button.enable'):
                    self.get_logger().info('Enabling robot.')
                    self.send_enable_request(True)
                if button_got_pressed('button.disable'):
                    self.get_logger().info('Disabling robot.')
                    self.send_enable_request(False)
            except IndexError:
                self.get_logger().error('Joystick message/config error: parameters call for more buttons than in message.')

        if (self.last_joy.buttons != msg.buttons) or (msg.axes[ (thr := self.get_param_val('axis.throttle').integer_value) ] != self.last_joy.axes[thr]):
            try:
                for i in range(self.get_param_val('rpm_override_count').integer_value):
                    override_namespace = 'rpm_override_' + str(i)
                    speed_multiplier = self.get_param_val(override_namespace + '.speed_multiplier').double_value
                    overrides_motors = self.get_param_val(override_namespace + '.overrides_motors').integer_array_value
                    if (direction := button_is_pressed(override_namespace + '.button_backward')) or button_is_pressed(override_namespace + '.button_forward'):
                        for i in overrides_motors:
                            self.rpm_overrides.data[abs(i)] = throttle * speed_multiplier * (-1 if direction else 1) * (-1 if i<0 else 1) * self.get_param_val('maximum.rpm').double_array_value[i]
                    if button_got_released(override_namespace + '.button_backward') or button_got_released(override_namespace + '.button_forward'):
                        for i in overrides_motors: self.rpm_overrides.data[abs(i)] = 0
            except IndexError:
                self.get_logger().error('Configuration error: rpm overrides array length error.')

        # save the most recent joystick message, so we can compare with it to figure out what moved
        self.last_joy = msg

    def maximum_velocity_callback(self, msg):
        self.set_parameters([
                Parameter('maximum.vel.linear',  Parameter.Type.DOUBLE_ARRAY, [ msg.linear.x,  msg.linear.y,  msg.linear.z  ]),
                Parameter('maximum.vel.angular', Parameter.Type.DOUBLE_ARRAY, [ msg.angular.x, msg.angular.y, msg.angular.z ]),
        ])

    def maximum_rpm_callback(self, msg):
        if len(msg.layout.dim) != 2:
            self.get_logger().warn('Maximum RPM error: data malformed. Expected 2 dimensions')

        self.set_parameters([
                Parameter('maximum.rpm', Parameter.Type.DOUBLE_ARRAY, array('d', msg.data))
        ])
        # copy override data sizes, but retain our set values
        tmp = self.rpm_overrides.data
        msg.data = [math.nan] * len(msg.data)
        self.rpm_overrides = msg
        self.rpm_overrides.data[0:len(tmp)] = tmp


    """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
    def timer_callback(self):
        self.twist_publisher_.publish(self.velocities)
        self.rpm_override_publisher_.publish(self.rpm_overrides)

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
            # receive enable request responses (they come in separately)
            if bento_teleop.future.done() and bento_teleop.future.result() is not None:
                #bento_teleop.get_logger().debug('Dis/Enable result: %s' % bento_teleop.future.result().success)
                if bento_teleop.future.result().success == False: bento_teleop.get_logger().warn('Dis/Enable request returned error. Check service server!')
                bento_teleop.future.set_result(None)

    # shut down cleanly
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)



if __name__ == '__main__':
    main()
