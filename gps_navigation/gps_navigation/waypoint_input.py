import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class WaypointInput(Node):

    def __init__(self):
        super().__init__('waypoint_input')
        self.publisher_ = self.create_publisher(PoseStamped, 'waypoint_input', 10)
        self.initial_pose = None
        self.subscription = self.create_subscription(
            PoseStamped,
            'waypoints',
            self.initial_pose_callback,
            10)
        self.get_logger().info('Waiting for initial pose...')
        self.get_logger().info('Ready to accept GPS coordinates. Type "exit" to quit.')

    def initial_pose_callback(self, msg):
        self.initial_pose = msg.pose
        self.get_logger().info(f'Initial pose set: lat={self.initial_pose.position.x}, lon={self.initial_pose.position.y}, alt={self.initial_pose.position.z}')
        self.get_logger().info('Enter GPS coordinates (latitude, longitude, altitude) or "exit":')

    def publish_waypoint(self, lat, lon, alt):
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = lat
        waypoint.pose.position.y = lon
        waypoint.pose.position.z = alt
        self.publisher_.publish(waypoint)
        self.get_logger().info(f'Published waypoint: lat={lat}, lon={lon}, alt={alt}')

def main(args=None):
    rclpy.init(args=args)
    waypoint_input = WaypointInput()

    while rclpy.ok():
        user_input = input('Enter GPS coordinates (latitude, longitude, altitude) or "exit": ')
        if user_input.lower() == 'exit':
            break

        try:
            lat, lon, alt = map(float, user_input.split(','))
            waypoint_input.publish_waypoint(lat, lon, alt)
        except ValueError:
            waypoint_input.get_logger().error('Invalid input. Please enter the coordinates as comma-separated values: latitude, longitude, altitude')

    waypoint_input.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

