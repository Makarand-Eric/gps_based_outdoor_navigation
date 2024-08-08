import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class SimpleNavigation(Node):

    def __init__(self):
        super().__init__('simple_navigation')
        self.subscription = self.create_subscription(
            PoseStamped,
            'waypoints',
            self.waypoint_callback,
            10)
        self.input_subscription = self.create_subscription(
            PoseStamped,
            'waypoint_input',
            self.input_waypoint_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your robot's odometry topic
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_pose = None
        self.target_pose = None
        self.initial_pose = None
        self.linear_pid = PIDController(0.5, 0.0, 0.1)
        self.angular_pid = PIDController(2.0, 0.0, 0.1)
        self.last_time = self.get_clock().now()
        self.gps_last_received = self.get_clock().now()  # Time when GPS data was last received
        self.gps_timeout = Duration(seconds=5)  # Timeout duration for GPS data
        self.get_logger().info('Simple Navigation Node has been started.')

    def waypoint_callback(self, msg):
        self.gps_last_received = self.get_clock().now()  # Update GPS data received time
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.get_logger().info(f'Setting initial pose: lat={self.initial_pose.position.x}, lon={self.initial_pose.position.y}, alt={self.initial_pose.position.z}')
            self.initial_pose.position.z = 0.0  # Set the initial altitude to 0.0 as a float
        else:
            self.get_logger().info('Initial pose already set, ignoring further waypoints from waypoint_publisher.')

    def input_waypoint_callback(self, msg):
        if self.initial_pose is None:
            self.get_logger().warn('Initial pose not set yet. Ignoring input waypoint.')
            return
        # Translate the input GPS coordinates relative to the initial pose
        self.target_pose = PoseStamped()
        self.target_pose.header = msg.header
        self.target_pose.pose.position.x = msg.pose.position.x - self.initial_pose.position.x
        self.target_pose.pose.position.y = msg.pose.position.y - self.initial_pose.position.y
        self.target_pose.pose.position.z = msg.pose.position.z - self.initial_pose.position.z
        self.get_logger().info(f'Received input waypoint: x={self.target_pose.pose.position.x}, y={self.target_pose.pose.position.y}, z={self.target_pose.pose.position.z}')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if self.current_pose and self.target_pose:
            # Check for GPS timeout
            current_time = self.get_clock().now()
            if (current_time - self.gps_last_received) > self.gps_timeout:
                self.get_logger().warn('GPS data timeout. Stopping the robot.')
                self.stop_robot()
                return

            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            twist_msg = Twist()

            # Calculate distance and angle to the target
            dx = self.target_pose.pose.position.x - self.current_pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)

            # Get current yaw
            current_yaw = self.get_yaw_from_pose(self.current_pose)

            # Calculate angular error
            angle_error = self.normalize_angle(angle_to_target - current_yaw)

            # PID controller for linear and angular velocity
            constant_speed = 0.5  # Set a constant speed
            linear_velocity = self.linear_pid.control(constant_speed, dt)
            angular_velocity = self.angular_pid.control(angle_error, dt)

            # If the robot is close to the target, reduce speed and stop
            if distance < 0.1:  # Threshold to stop the bot
                linear_velocity = 0.0
                angular_velocity = 0.0
                self.target_pose = None  # Clear the target waypoint

            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity

            self.publisher_.publish(twist_msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    simple_navigation = SimpleNavigation()
    rclpy.spin(simple_navigation)
    simple_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

