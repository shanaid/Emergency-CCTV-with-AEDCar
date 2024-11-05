import rclpy
from rclpy.node import Node
from safety_package.qos import qos_service

from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import tf2_ros
from math import pi, cos, sin

class odom(Node):

    def __init__(self):
        super().__init__('odom')
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.turtlebot_callback, qos_service)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', qos_service)
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.odom_msg = Odometry()
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.is_status = False
        self.is_calc_theta = False

        self.x = 0
        self.y = 0
        self.theta = 0
        self.prev_time = 0
        self.omega_max = 11.7

        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = 'base_link'

        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform.header.frame_id = 'base_link'
        self.laser_transform.child_frame_id = 'laser'
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 1.0
        self.laser_transform.transform.rotation.w = 1.0

        self.linear_x = 0.0
        self.curr_angular_z = 0.0

    def turtlebot_callback(self, msg) :
        if self.is_status == False :
            self.is_status = True
            self.x = msg.twist.angular.x
            self.y = msg.twist.angular.y
            self.prev_time = rclpy.clock.Clock().now()
        else : 
            self.linear_x = msg.twist.linear.x
            # self.target_angular_velocity = -msg.twist.angular.z*self.omega_max
            self.angular_z = -msg.twist.angular.z
            self.linear_z = (msg.twist.linear.z / 180) * pi

            self.current_time = rclpy.clock.Clock().now()
            self.period = (self.current_time - self.prev_time).nanoseconds/1000000000

            # self.angular_z = self.curr_angular_z + 5 * (self.target_angular_velocity - self.curr_angular_z) * self.period
           
            self.x += self.linear_x*cos(self.theta)*self.period
            self.y += self.linear_x*sin(self.theta)*self.period
            # self.theta += self.angular_z*self.period
            self.theta = self.linear_z

            q = Quaternion.from_euler(0, 0, self.theta)
            self.base_link_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.laser_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.base_link_transform.transform.translation.x = self.x
            self.base_link_transform.transform.translation.y = self.y
            self.base_link_transform.transform.rotation.x = q.x
            self.base_link_transform.transform.rotation.y = q.y
            self.base_link_transform.transform.rotation.z = q.z
            self.base_link_transform.transform.rotation.w = q.w

            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.orientation.x = q.x
            self.odom_msg.pose.pose.orientation.y = q.y
            self.odom_msg.pose.pose.orientation.z = q.z
            self.odom_msg.pose.pose.orientation.w = q.w
            self.odom_msg.twist.twist.linear.x = self.linear_x
            self.odom_msg.twist.twist.angular.z = self.angular_z

            self.broadcaster.sendTransform(self.base_link_transform)
            self.broadcaster.sendTransform(self.laser_transform)

            self.odom_publisher.publish(self.odom_msg)
            self.prev_time = self.current_time
            # self.curr_angular_z = self.angular_z


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = odom()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
