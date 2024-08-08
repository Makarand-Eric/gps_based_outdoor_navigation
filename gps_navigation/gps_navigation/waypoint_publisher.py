import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from geometry_msgs.msg import PoseStamped

class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.get_logger().info('Initializing waypoint_publisher node...')
        self.subscription_navsat = self.create_subscription(
            NavSatFix,
            '/mavros/mavros/gps1/raw',
            self.listener_navsat_callback,
            10)
        self.subscription_gpsraw = self.create_subscription(
            GPSRAW,
            '/mavros/mavros/gps1/raw',
            self.listener_gpsraw_callback,
            10)
        self.publisher_ = self.create_publisher(PoseStamped, 'waypoints', 10)
        self.get_logger().info('Subscribed to /mavros/mavros/gps1/raw and ready to publish on /waypoints')

    def listener_navsat_callback(self, msg):
        self.get_logger().info(f'Received NavSatFix GPS data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')
        self.publish_waypoint(msg.latitude, msg.longitude, msg.altitude)

    def listener_gpsraw_callback(self, msg):
        self.get_logger().info(f'Received GPSRAW GPS data: lat={msg.lat}, lon={msg.lon}, alt={msg.alt}')
        self.publish_waypoint(msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1e3)

    def publish_waypoint(self, lat, lon, alt):
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = lat
        waypoint.pose.position.y = lon
        waypoint.pose.position.z = alt
        self.publisher_.publish(waypoint)
        self.get_logger().info(f'Published waypoint: x={lat}, y={lon}, z={alt}')

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

