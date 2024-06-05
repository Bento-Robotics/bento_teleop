import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

#TODO: Call enable (bool) service
#TODO: Wait until Joy listener receives stuff before publishing Twist
#TODO: Check if joy msg has enough axes for our parameters → no array overflow
#TODO: Subscribe to maximums and map accordingly

class Bento_Teleop(Node):
    velocities = Twist()
    last_joy = Joy()

    def __init__(self):
        super().__init__('bento_teleop')
        self.subscription_ = self.create_subscription(Joy, '/teleop/joy', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.declare_parameter('axis_linear',   1)
        self.declare_parameter('axis_angular',  2)
        self.declare_parameter('axis_throttle', 3)
        self.declare_parameter('button_enable',  2)
        self.declare_parameter('button_disable', 3)
        self.declare_parameter('publish_rate', 0.1) # seconds
        self.timer = self.create_timer(self.get_parameter('publish_rate').get_parameter_value().double_value, self.timer_callback)
        #self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        if msg.header.stamp.sec < self.last_joy.header.stamp.sec:
            self.get_logger().warn('Joystick message time travel detected. Check your publishers.')
        #self.get_logger().info('YOOOOOOO "%f"' % msg.axes[ self.get_parameter('axis_throttle').get_parameter_value().integer_value ] )
        throttle = self.scale(msg.axes[ self.get_parameter('axis_throttle').get_parameter_value().integer_value ], -1.0, 1.0, 0.0, 1.0)
        self.velocities.linear.x = self.scale(msg.axes[ self.get_parameter('axis_linear').get_parameter_value().integer_value ] * throttle, -1.0, 1.0, -10.0, 10.0)
        self.velocities.angular.z = self.scale(msg.axes[ self.get_parameter('axis_angular').get_parameter_value().integer_value ] * throttle, -1.0, 1.0, -10.0, 10.0)
        self.last_joy = msg

    def timer_callback(self):
        msg = self.velocities
        self.publisher_.publish(msg)

    def scale(self, num, inMin, inMax, outMin, outMax):
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))


def main(args=None):
    rclpy.init(args=args)

    bento_teleop = Bento_Teleop()

    rclpy.spin(bento_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bento_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
