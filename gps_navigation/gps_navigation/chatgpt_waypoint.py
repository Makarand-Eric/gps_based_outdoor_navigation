import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import math

class SinglePointNavigator(Node):

    def __init__(self):
        super().__init__('single_point_navigator')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define the goal point
        self.goal_x = 7.0
        self.goal_y = 1.5
        
        # Current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Flag to stop the robot when the goal is reached
        self.goal_reached = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_euler(orientation_q)

        if not self.goal_reached:
            self.navigate_to_goal()

    def quaternion_to_euler(self, q):
        # Convert quaternion to euler yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def navigate_to_goal(self):
        # Calculate distance and angle to the goal
        distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        # Calculate angle difference
        angle_diff = angle_to_goal - self.current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Create Twist message to move the robot
        twist = Twist()
        if distance > 0.1:  # Distance threshold to stop the robot near the goal
            twist.linear.x = 0.2
            twist.angular.z = 0.5 * angle_diff
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.goal_reached = True
            self.get_logger().info('Goal reached!')

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SinglePointNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

