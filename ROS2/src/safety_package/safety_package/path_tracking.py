import rclpy
import cv2
import base64

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point, Point32, Pose, PoseStamped
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2, pow
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Int16

# path_tracking 노드는 로봇의 위치(/odom), 로봇의 속도(/turtlebot_status), 주행 경로(/local_path)를 받아서, 주어진 경로를 따라가게 하는 제어 입력값(/cmd_vel)을 계산합니다.
# 제어입력값은 선속도와 각속도로 두가지를 구합니다. 


# 노드 로직 순서
# 1. 제어 주기 및 타이머 설정
# 2. 파라미터 설정
# 3. Quaternion 을 euler angle 로 변환
# 4. 터틀봇이 주어진 경로점과 떨어진 거리(lateral_error)와 터틀봇의 선속도를 이용해 전방주시거리 계산
# 5. 전방 주시 포인트 설정
# 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산
# 7. 선속도, 각속도 정하기

global auto_mode_info, is_trigger

auto_mode_info = False
is_trigger = False
diary_regist_li = []
diary_regist = {
    'plant_original_name' : 'None',
    'plant_img' : 'None',
}

class followTheCarrot(Node):
    def __init__(self):
        super().__init__('path_tracking') 

        # 로봇을 움직이게 하는 부분
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 로봇의 현재 위치
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)

        # 경로 받아오기
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)

        # 라이다 데이터 구독
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 로직 1. 제어 주기 및 타이머 설정
        time_period=0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        # handcontrol node에 제어 메시지를 보냄
        self.hand_control_pub = self.create_publisher(Int16, '/hand_control_cmd', 10)

        # 목표 좌표를 가지고 옴
        # self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback, 1)

        # a_star에 목표 좌표를 보냄
        self.a_star_goal_pub = self.create_publisher(Point, '/a_star_goal', 10)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_forward_approach = False
        self.is_right_approach = False
        self.is_left_approach = False
        self.is_trigger = True

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.turtle_yaw = 0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        self.handcontrol_cmd_msg = Int16()

        # 로직 2. 파라미터 설정(전방주시거리)
        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=1.0

        # 터틀봇이 정지해있는지 판단
        self.stop_cnt = 0 # 터틀봇이 멈춰 있는지 판단하는 간격을 정하기 위한 변수
        self.is_stop = False # 터틀봇의 정지 여부 판단
        self.out_vel = 0.0 # 탈출을 계속하기 위해 필요함
        self.out_rad_vel = 0.0

        self.turn_cnt = 0
        self.go_cnt = 0
        self.back_cnt = 0

        # 라이다 주변 접근 정보
        self.forward_dis = 0
        self.left_dis = 0
        self.right_dis = 0

         # 터틀봇의 현재 위치
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        self.omega_max = 11.7

        
        # trigger 정보
        self.goal_x = 0
        self.goal_y = 0
        self.plant_original_name = ''
        self.palnt_number = 0
        self.triggers_idx = 0   # 현위치에서 가까운 화분의 인덱스
        self.visited = set()
        self.sun_spot_x = 0
        self.sun_spot_y = 0
       

        # 카메라 센서 이미지
        self.original_img = None

        # 물주기 기능에 사용되는 변수
        self.pickture = set()   # 사진은 한번만
        self.water_time = 0     # 물 주는 동안 기다림
        self.check_stop = 0     # 멈췄는 지 확인 하는 함수
        self.is_yolo_finish = False  # 물주기 끝내는 변수

        # 화분 이동 기능에 사용되는 변수
        self.hand_control_msg = Int16() # handcontrol에 모드를 보냄       
        self.is_lift = False    # 해당 장소에 대기하고 있는 시간
        self.wait_time = 0
        self.lift_idx = 0
        self.is_close = False
        self.is_goal_wait = False

        # 물 주기 완료 후 백으로 다시 전달하는 변수
        self.diary_regist = {
            'plant_original_name' : 'None',
            'plant_img' : 'None',
        }   

    def timer_callback(self):

        if self.is_status and self.is_odom and self.is_path and self.is_lidar:
            
            if len(self.path_msg.poses) > 1:

                self.is_look_forward_point = False

                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x - robot_pose_x, 2) + pow(self.path_msg.poses[0].pose.position.y - robot_pose_y, 2))

                self.lfd = (self.status_msg.twist.linear.x + lateral_error)*0.5
                if self.lfd < self.min_lfd :
                    self.lfd = self.min_lfd
                if self.lfd > self.max_lfd :
                    self.lfd = self.max_lfd

                min_dis = float('inf')

                for num, waypoint in enumerate(self.path_msg.poses) :
                    self.current_point = waypoint.pose.position

                    dis = sqrt(pow(self.path_msg.poses[0].pose.position.x - self.current_point.x, 2) + pow(self.path_msg.poses[0].pose.position.y - self.current_point.y, 2))
                    if abs(dis-self.lfd) < min_dis :
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = self.current_point
                        self.is_look_forward_point = True

                if self.is_look_forward_point :
                    
                    global_forward_point = [self.forward_point.x, self.forward_point.y, 1]

                    trans_matrix = np.array([
                                            [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                                            [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                                            [0, 0, 1]
                                            ])
                    
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    theta = -atan2(local_forward_point[1], local_forward_point[0])

                    # 선속, 각속 값 지정
                    out_vel = 1.0
                    out_rad_vel = theta
                    
                    if len(self.path_msg.poses) < 20:
                        out_vel = 0.7
                        if len(self.path_msg.poses) < 10:
                            outvel = 0.3
                        out_rad_vel = theta

                    self.cmd_msg.linear.x = out_vel
                    self.cmd_msg.angular.z = out_rad_vel/self.omega_max

            else: 
                if self.goal_x - 1 <= self.robot_pose_x <= self.goal_x + 1 and self.goal_y - 1 <= self.robot_pose_y <= self.goal_y + 1:
                    # print("goal")
                    self.cmd_msg.linear.x=0.0
                    self.cmd_msg.angular.z=0.0
                    self.is_goal_wait = True

            if self.is_goal_wait:
                print("aa")
                self.wait_time += 1
                if self.wait_time >= 100:
                    self.wait_time = 0
                    self.is_goal_wait = False
                    goal = Point()
                    goal.x, goal.y = 0, 0
                    self.a_star_goal_pub.publish(goal)
            
            else:
                # 전방에 장애물이 있다면, 뒤로 후진
                if self.forward_dis <= 0.05:
                    self.cmd_msg.linear.x = -0.5
                    self.cmd_msg.angular.z = 0.0
                elif self.left_dis <= 0.1:
                    self.cmd_msg.linear.x = -0.5
                    self.cmd_msg.angular.z = -120.0
                elif self.right_dis <= 0.1:
                    self.cmd_msg.linear.x = -0.5
                    self.cmd_msg.angular.z = -60.0


        self.cmd_pub.publish(self.cmd_msg)
                  
 


    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg

        # 로직 3. Quaternion 을 euler angle 로 변환
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
                       msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()


    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

    # 라이다 데이터 수신시 호출되는 콜백함수
    def lidar_callback(self, msg):
        self.is_lidar = True
        self.lidar_msg = msg
        # 경로와 위치를 알고 있어야 하기 때문에 알고 있는지 체크
        if self.is_path == True and self.is_odom == True:
            # 직교좌표계 데이터를 가질 포인트클라우드 생성
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id = 'map'
            # 로컬 to 글로벌 변환 행렬
            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
            t = np.array([[cos(theta), -sin(theta), pose_x],
                          [sin(theta), cos(theta), pose_y],
                          [0, 0, 1]])
            # 극좌표계를 직교좌표계로 만들기
            for angle, r in enumerate(msg.ranges):
                global_point = Point32()

                if 0.0 < r < 12:
                    # 극좌표계를 로봇 기준(로컬) 직교좌표계로
                    local_x = r * cos(angle * pi / 180)
                    local_y = r * sin(angle * pi / 180)
                    local_point = np.array([[local_x], [local_y], [1]])
                    # 로컬 직교좌표계를 맵 기준(글로벌) 직교좌표계로
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    # 포인트 클라우드에 맵 기준 직교좌표계 데이터 추가
                    # 퍼블리시는 하지 않았지만 확인하고 싶으면 pcd_msg를 퍼블리시해서 rviz에서 확인할 것
                    pcd_msg.points.append(global_point)

            # 전/후방, 좌/우측 충돌 감지
            forward_left = self.lidar_msg.ranges[0:1]
            forward_right = self.lidar_msg.ranges[359:360]
            forward = forward_left + forward_right
            backward = self.lidar_msg.ranges[170:191]
            left = self.lidar_msg.ranges[20:31]
            right = self.lidar_msg.ranges[330:341]

            # 평균 거리 계산
            self.forward_dis = sum(forward) / len(forward)
            backward_dis = sum(backward) / len(backward)
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

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)


    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() # 선속도와 각속도로 두가지를 구합니다. 