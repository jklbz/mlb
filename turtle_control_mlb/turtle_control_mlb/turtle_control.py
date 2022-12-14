import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math


class TurtleControl(Node):

    def __init__(self):
        super().__init__('turtle_control')
        self.init_variables()
        self.init_subscribers()
        self.init_publisher()

    def init_publisher(self):
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)

    def init_subscribers(self):
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)

        self.subscription = self.create_subscription(
            Pose2D,
            'goal',
            self.goal_callback,
            10)

        self.subscription  # prevent unused variable warning

    def init_variables(self):
        self.x = 0.0
        self.x_error = 0.0
        self.x_goal = 0.0
        self.y = 0.0
        self.y_error = 0.0
        self.y_goal = 0.0
        self.v_max = 1.0
        self.k_omega = 1.0
        self.theta = 0.0
        self.p = 0.0
        self.alpha = 0.0

    def pose_callback(self, msg):
        # callback para receber a pose do turtlesim;
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def goal_callback(self, msg):
        #callback para receber a posição objetivo;
        self.x_goal = msg.x
        self.y_goal = msg.y

    def pub_callback(self):
        # método principal do nó, implementado como callback do publisher.
        self.x_error = self.x_goal - self.x
        self.y_error = self.y_goal - self.y
        
        if((math.sqrt(self.x_error**2) > 0.1) or (math.sqrt(self.y_error**2) > 0.1)):
            self.p = math.sqrt((self.x_error**2) + (self.y_error**2))
            self.alpha = math.atan2(self.y_error,self.x_error) - (self.theta)
        else:
            self.p = 0.0
            self.alpha = 0.0

        msg = Twist()
        msg.linear.x = self.v_max * math.tanh(self.p)
        msg.linear.y = 0.0
        msg.angular.z = self.k_omega * self.alpha

        self.publisher_.publish(msg)

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
