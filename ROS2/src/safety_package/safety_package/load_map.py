import rclpy
import numpy as np
from rclpy.node import Node
from safety_package.qos import qos_service

import os
from geometry_msgs.msg import Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from math import pi

# load_map 노드는 맵 데이터를 읽어서, 맵 상에서 점유영역(장애물) 근처에 로봇이 움직일 수 없는 영역을 설정하고 맵 데이터로 publish 해주는 노드입니다.
# 추 후 a_star 알고리즘에서 맵 데이터를 subscribe 해서 사용합니다.

# 노드 로직 순서
# 1. 맵 파라미터 설정
# 2. 맵 데이터 읽고, 2차원 행렬로 변환
# 3. 점유영역 근처 필터처리

class loadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub2 = self.create_publisher(OccupancyGrid, 'map', qos_service)
        time_period=1
        self.timer = self.create_timer(time_period, self.timer_callback)
       
        # 로직 1. 맵 파라미터 설정
        # 제공한 맵 데이터의 파라미터입니다. size_x,y는 x,y 방향으로 grid의 개수이고, resolution은 grid 하나당 0.05m라는 것을 의미합니다.
        # offset_x,y 의 -8, -4 는 맵 데이터가 기준 좌표계(map)로 부터 떨어진 거리를 의미합니다. 
        # 각 항에 -8.75를 뺀이유는 ros에서 occupancygrid의 offset이라는 데이터는 맵의 중앙에서 기준좌표계까지 거리가 아니라 맵의 우측하단에서 부터 기준좌표계까지의 거리를 의미합니다.
        # 따라서 (350*0.05)/2를 해준 값을 빼줍니다.
        self.map_msg2=OccupancyGrid()

        self.map_size_x=500
        self.map_size_y=500
        self.map_resolution=0.05
        self.map_offset_x=-47.0-12.5
        self.map_offset_y=-58.0-12.5

        self.map_data2 = [0 for i in range(self.map_size_x*self.map_size_y)]
        grid2=np.array(self.map_data2)
        grid2=np.reshape(grid2,(self.map_size_x, self.map_size_y))

        self.map_msg2.header.frame_id="map"

   

        m = MapMetaData()
        m.resolution = self.map_resolution
        m.width = self.map_size_x
        m.height = self.map_size_y
        m.origin = Pose()
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.map_meta_data = m

        self.map_msg2.info=self.map_meta_data
        
        # 로직 2. 맵 데이터 읽고, 2차원 행렬로 변환

        pkg_path = os.getcwd()
        back_floder = '..'
        folder_name = 'map'

        file_name2 = 'map2.txt'
        full_path2=os.path.join(pkg_path, back_floder, folder_name, file_name2)
        self.f2=open(full_path2, 'r')
        
        line2 = self.f2.readline()
        line_data2 = line2.split()
        
        for num,data in enumerate(line_data2) :
            self.map_data2[num]=int(data)
   
        map_to_grid2=np.array(self.map_data2)
        grid2=np.reshape(map_to_grid2, (self.map_size_x, self.map_size_y))


        for y in range(self.map_size_y):
            for x in range(self.map_size_x):
                if grid2[x][y] == 100 :

                    # 로직 3. 점유영역 근처 필터처리
                    for c in range(-5, 6) :
                        for r in range(-5, 6) :
                            if 0 <= x+r < self.map_size_x and 0 <= y+c < self.map_size_y and grid2[x+r][y+c] < 80 :
                                grid2[x+r][y+c] = 127

        np_map_data2=grid2.reshape(1,self.map_size_x*self.map_size_y) 
        list_map_data2=np_map_data2.tolist()
   
   
        ## 로직2를 완성하고 주석을 해제 시켜주세요.
        self.f2.close()
        print('read_complete')
        self.map_msg2.data=list_map_data2[0]


    def timer_callback(self):
        self.map_msg2.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_pub2.publish(self.map_msg2)

       
def main(args=None):
    rclpy.init(args=args)

    load_map = loadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()