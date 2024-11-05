import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from squaternion import Quaternion
import numpy as np
from math import pi, cos, sin, sqrt, atan2

class laserDis(Node):

    def __init__(self):
        super().__init__('lla')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        time_period=0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_lidar = False
        self.is_odom = False
        self.lidar_msg = LaserScan()
        self.odom_msg = Odometry()
        self.robot_yaw = 0.0
        self.forward_dis = 0.0
        self.left_dis = 0.0
        self.right_dis = 0.0

    def timer_callback(self):
        if self.is_lidar and self.is_odom:
            pass
            # print()

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _,_,self.robot_yaw = q.to_euler()

    def lidar_callback(self, msg):
        if self.is_odom:
            self.lidar_msg = msg
            self.is_lidar = True
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id = 'map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
        
            t = np.array([
                        [cos(theta), -sin(theta), pose_x],
                        [sin(theta), cos(theta), pose_y],
                        [0, 0, 1]
                        ])
            for angle, r in enumerate(msg.ranges) :
                global_point = Point32()

                if 0.0 < r < 12 :
                    local_x = r*cos(angle*pi/180)
                    local_y = r*sin(angle*pi/180)
                    local_point = np.array([
                                            [local_x],
                                            [local_y],
                                            [1]
                                        ])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)

            forward_left = self.lidar_msg.ranges[0:2]
            forward_right = self.lidar_msg.ranges[358:360]
            forward = forward_left + forward_right
            left = self.lidar_msg.ranges[20:31]
            right = self.lidar_msg.ranges[330:341]

            # 평균으로 이용하지 말고, 최대값 혹은 최소값으로 이용하면?
            print(f"ranges: {self.lidar_msg.ranges[0:2]}, {self.lidar_msg.ranges[358:360]}")
            self.forward_dis = sum(forward) / len(forward)
            self.left_dis = sum(left) / len(left)
            self.right_dis = sum(right) / len(right)

            # 근접 감지
            if self.forward_dis < 0.05:
                self.is_forward_approach = True
                print('전방 근접')
            else:
                self.is_forward_approach = False 

            if self.left_dis < 0.1:
                self.is_left_approach = True
                print('좌측 근접')
            else:
                self.is_left_approach = False

            if self.right_dis < 0.1:
                self.is_right_approach = True
                print('우측 근접')
            else:
                self.is_right_approach = False
            


def main(args=None):
    rclpy.init(args=args)

    laser_dis = laserDis()

    rclpy.spin(laser_dis)


    laser_dis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
