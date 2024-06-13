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

    def __init__(self):
        super().__init__('bento_teleop')

        # initialize parameters
        self.declare_parameter('axis_linear',   1)
        self.declare_parameter('axis_angular',  2)
        self.declare_parameter('axis_throttle', 3)
        self.declare_parameter('button_enable',  2)
        self.declare_parameter('button_disable', 3)
        self.declare_parameter('publish_rate', 0.1)  # seconds
        self.declare_parameter('robot_namespace', '/bento')
        self.declare_parameter('enable_service_name', '/enable')
        self.declare_parameter('maximum_rpm_topic', '/maximum/rpms')
        self.declare_parameter('maximum_vel_topic', '/maximum/velocity')
        self.declare_parameter('maximum_rpm_default', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('maximum_vel_default.linear', [10.0, 0.0, 0.0])
        self.declare_parameter('maximum_vel_default.angular', [10.0, 0.0, 0.0])

        # initialize subscribers, subscribers, timers and service clients
        self.joy_subscription_    = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.maxRPM_subscription_ = self.create_subscription(Float32MultiArray, ( self.get_parameter('robot_namespace').get_parameter_value().string_value + self.get_parameter('maximum_rpm_topic').get_parameter_value().string_value ), self.maximum_rpm_callback, 10)
        self.maxVel_subscription_ = self.create_subscription(Twist, ( self.get_parameter('robot_namespace').get_parameter_value().string_value + self.get_parameter('maximum_vel_topic').get_parameter_value().string_value ), self.maximum_velocity_callback, 10)
        self.twist_publisher_     = self.create_publisher(Twist, ( self.get_parameter('robot_namespace').get_parameter_value().string_value + '/cmd_vel' ), 10)
        self.timer                = self.create_timer(self.get_parameter('publish_rate').get_parameter_value().double_value, self.timer_callback)
        self.enable_client        = self.create_client(SetBool, ( self.get_parameter('robot_namespace').get_parameter_value().string_value + self.get_parameter('enable_service_name').get_parameter_value().string_value ))

        # make sure the service exists already, and send a disable for good measure
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('enable service not available yet, waiting...')
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
        if msg.header.stamp.sec < self.last_joy.header.stamp.sec:
            self.get_logger().warn('Joystick message time travel detected. Check your publishers.')

        # get joystick axes, modify linear and angular with throttle
        try:
            #self.get_logger().debug('Axis test: "%f"' % msg.axes[self.get_parameter('axis_throttle').get_parameter_value().integer_value] )
            throttle = self.scale(msg.axes[self.get_parameter('axis_throttle').get_parameter_value().integer_value], -1.0, 1.0, 0.0, 1.0)
            max_linear = self.get_parameter('maximum_vel_default.linear').get_parameter_value().double_array_value[0]
            self.velocities.linear.x = self.scale(msg.axes[self.get_parameter('axis_linear').get_parameter_value().integer_value] * throttle, -1.0, 1.0, -max_linear, max_linear)
            max_angular = self.get_parameter('maximum_vel_default.angular').get_parameter_value().double_array_value[0]
            self.velocities.angular.z = self.scale(msg.axes[self.get_parameter('axis_angular').get_parameter_value().integer_value] * throttle, -1.0, 1.0, -max_angular, max_angular)
        except IndexError:
            self.get_logger().error('Joystick message/config error: parameters call for more axes than in message.')

        # get joystick buttons, and send enable/disable requests accordingly
        try:
            if (msg.buttons[self.get_parameter('button_enable').get_parameter_value().integer_value] == 1
                    and not self.last_joy.buttons[self.get_parameter('button_enable').get_parameter_value().integer_value] == 1):
                self.get_logger().info('Enabling robot.')
                self.send_enable_request(True)
            if (msg.buttons[self.get_parameter('button_disable').get_parameter_value().integer_value] == 1
                    and not self.last_joy.buttons[self.get_parameter('button_disable').get_parameter_value().integer_value] == 1):
                self.get_logger().info('Disabling robot.')
                self.send_enable_request(False)
        except IndexError:
            self.get_logger().error('Joystick message/config error: parameters call for more buttons than in message.')

        # save the most recent joystick message, so we can compare with it to figure out what moved
        self.last_joy = msg

    def maximum_velocity_callback(self, msg):
        self.set_parameters([
                Parameter('maximum_vel_default.linear', rclpy.Parameter.Type.DOUBLE_ARRAY, [ msg.linear.x, msg.linear.y, msg.linear.z ]),
                Parameter('maximum_vel_default.angular', rclpy.Parameter.Type.DOUBLE_ARRAY, [ msg.angular.x, msg.angular.y, msg.angular.z ])
        ])

    def maximum_rpm_callback(self, msg):
        if len(msg.layout.dim) != 2:
            self.get_logger().warn('Maximum RPM error: data malformed. Expected 2 dimensions')

        data = []
        for i in range(len(msg.data) // (msg.layout.dim[1].stride // msg.layout.dim[1].size)):
            data.append(msg.data[msg.layout.data_offset + i * msg.layout.dim[1].stride])
        self.set_parameters([
                Parameter('maximum_rpm_default', rclpy.Parameter.Type.DOUBLE_ARRAY, data)
        ])


    """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
    def timer_callback(self):
        msg = self.velocities
        self.twist_publisher_.publish(msg)

    """scale inputs min/max values, used for mapping joystick data"""
    def scale(self, num, inMin, inMax, outMin, outMax):
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))



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
