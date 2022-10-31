import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TurtleControl(Node):

    def __init__(self):
        super().__init__('turtle_control')
        self.init_publisher()

    def init_publisher(self):
        self.publisher_ = self.create_publisher(String, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.i = 0

    def init_subscribers(self):
        #inicialize seus subscribers,
        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.pose_callback,
            10)

        self.subscription = self.create_subscription(
            String,
            'goal',
            self.goal_callback,
            10)
        self.subscription  # prevent unused variable warning

    def init_variables(self):
        self.x = 0
        self.x_error = 0
        self.x_goal = 0
        self.k_omega = 0

    def pose_callback(self, msg):
        # callback para receber a pose do turtlesim;
        self.get_logger().info('pose: "%s"' % msg.data)

    def goal_callback(self, msg):
        #callback para receber a posição objetivo;
        self.get_logger().info('goal: "%s"' % msg.data)

    def pub_callback(self):
        # método principal do nó, implementado como callback do publisher.
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
#        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    turtle_control = TurtleControl()

    rclpy.spin(turtle_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
