import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

# TODO: Wait until Joy listener receives stuff before publishing Twist


class Bento_Teleop(Node):
    velocities = Twist()
    last_joy = Joy()
    rpmOverrides = Float32MultiArray()
    rpmOverrides.layout.data_offset = 0
    dim1 = MultiArrayDimension()
    dim1.label ='controllers'
    dim1.size = 4
    dim1.stride = 2
    rpmOverrides.layout.dim.append(dim1)
    dim2 = MultiArrayDimension()
    dim2.label ='motors'
    dim2.size = 2
    dim2.stride = 2
    rpmOverrides.layout.dim.append(dim2)

    def __init__(self):
        super().__init__('bento_teleop')

        # initialize parameters
        self.declare_parameter('axis_linear',   1)
        self.declare_parameter('axis_angular',  2)
        self.declare_parameter('axis_throttle', 3)
        self.declare_parameter('button_enable',  2)
        self.declare_parameter('button_disable', 3)
        self.declare_parameter('button_flippers_front_up', 9)
        self.declare_parameter('button_flippers_front_down', 8)
        self.declare_parameter('button_flippers_back_up', 11)
        self.declare_parameter('button_flippers_back_down', 10)
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
        self.rpmOverride_publisher_ = self.create_publisher(Float32MultiArray, ( self.get_parameter('robot_namespace').get_parameter_value().string_value + '/rpmOverride' ), 10)
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

        #self.get_logger().debug('Axis test: "%f"' % msg.axes[self.get_parameter('axis_throttle').get_parameter_value().integer_value] )
        throttle = self.scale(msg.axes[self.get_parameter('axis_throttle').get_parameter_value().integer_value], -1.0, 1.0, 0.0, 1.0)

        # get joystick axes, modify linear and angular with throttle
        try:
            max_linear = self.get_parameter('maximum_vel_default.linear').get_parameter_value().double_array_value[0]
            lin_x = self.scale(msg.axes[self.get_parameter('axis_linear').get_parameter_value().integer_value] * throttle, -1.0, 1.0, -max_linear, max_linear)
            if lin_x == 0.0: lin_x = 0.00000000000000000000000000001
            self.velocities.linear.x = lin_x
            max_angular = self.get_parameter('maximum_vel_default.angular').get_parameter_value().double_array_value[0]
            ang_y = self.scale(msg.axes[self.get_parameter('axis_angular').get_parameter_value().integer_value] * throttle, -1.0, 1.0, -max_angular, max_angular)
            if ang_y == 0.0: ang_y = 0.00000000000000000000000000001
            self.velocities.angular.z = ang_y
        except IndexError:
            self.get_logger().error('Joystick message/config error: parameters call for more axes than in message.')


        if (length := len( self.get_parameter('maximum_rpm_default').get_parameter_value().double_array_value )) is not len( self.rpmOverrides.data ):
            self.rpmOverrides.data = [0.0] * length

        # get joystick buttons, control flippers, and send enable/disable requests accordingly
        try:
            if (msg.buttons[self.get_parameter('button_enable').get_parameter_value().integer_value] == 1
                    and not self.last_joy.buttons[self.get_parameter('button_enable').get_parameter_value().integer_value] == 1):
                self.get_logger().info('Enabling robot.')
                self.send_enable_request(True)
            if (msg.buttons[self.get_parameter('button_disable').get_parameter_value().integer_value] == 1
                    and not self.last_joy.buttons[self.get_parameter('button_disable').get_parameter_value().integer_value] == 1):
                self.get_logger().info('Disabling robot.')
                self.send_enable_request(False)

            if ((button_value := msg.buttons[self.get_parameter('button_flippers_front_up').get_parameter_value().integer_value])
                    is not self.last_joy.buttons[self.get_parameter('button_flippers_front_up').get_parameter_value().integer_value]):
                self.rpmOverrides.data[0] = self.rpmOverrides.data[1] = \
                button_value * throttle * self.get_parameter('maximum_rpm_default').get_parameter_value().double_array_value[0]
            if ((button_value := msg.buttons[self.get_parameter('button_flippers_front_down').get_parameter_value().integer_value]) 
                    is not self.last_joy.buttons[self.get_parameter('button_flippers_front_down').get_parameter_value().integer_value]):
                self.rpmOverrides.data[0] = self.rpmOverrides.data[1] = \
                - button_value * throttle * self.get_parameter('maximum_rpm_default').get_parameter_value().double_array_value[1]

            if ((button_value := msg.buttons[self.get_parameter('button_flippers_back_up').get_parameter_value().integer_value])
                    is not self.last_joy.buttons[self.get_parameter('button_flippers_back_up').get_parameter_value().integer_value]):
                self.rpmOverrides.data[2] = self.rpmOverrides.data[3] = \
                button_value * throttle * self.get_parameter('maximum_rpm_default').get_parameter_value().double_array_value[2]
            if ((button_value := msg.buttons[self.get_parameter('button_flippers_back_down').get_parameter_value().integer_value]) 
                    is not self.last_joy.buttons[self.get_parameter('button_flippers_back_down').get_parameter_value().integer_value]):
                self.rpmOverrides.data[2] = self.rpmOverrides.data[3] = \
                - button_value * throttle * self.get_parameter('maximum_rpm_default').get_parameter_value().double_array_value[3]


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
        #for i in range(len(msg.data) // (msg.layout.dim[1].stride // msg.layout.dim[1].size)):
            #data.append(msg.data[msg.layout.data_offset + i * msg.layout.dim[1].stride])
        for i in range(len(msg.data)):
            data.append(msg.data[msg.layout.data_offset + i])

        self.set_parameters([
                Parameter('maximum_rpm_default', rclpy.Parameter.Type.DOUBLE_ARRAY, data)
        ])


    """timer callback, publishes Twist (cmd_vel) messages created from joystick data"""
    def timer_callback(self):
        msg = self.velocities
        self.twist_publisher_.publish(msg)
        self.rpmOverride_publisher_.publish(self.rpmOverrides)

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
