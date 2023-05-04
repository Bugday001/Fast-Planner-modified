#! /usr/bin/env python
# -*- coding: UTF-8 -*-
"""
给fast-planner依次发布目标点坐标
"""
import time
import re
import math
import rospy
import tf
from rospkg import RosPack
import numpy as np
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from std_msgs.msg import Empty

LAND = 0
FLY = 1
isGuolu = True  # 是否飞锅炉模型
class sendPath():
    def __init__(self):
        self.i = 0
        self.th = 0.30
        self.state = FLY
        if isGuolu:
            self.pathList = [
                            # [0, 3, 3, math.pi/16],
                            # [0, 6, 3, math.pi/8],
                            # [0, 9, 3, math.pi/4],
                            [0, 12, 3, math.pi/2],
                            [12, 12, 20, math.pi/2],
                            [12, -8, 20, 0],
                            [ -12,-8, 20, -math.pi/2],
                            [ -12,12, 20, math.pi],
                            [0, 12, 20, math.pi/2],
                            [12, 12, 30, math.pi/2],
                            [12, -8, 30, 0],
                            [ -12,-8, 30, -math.pi/2],
                            [-12,12,  30, math.pi],
                            [0, 0, 1, math.pi/2]]
            self.pathList = np.array(self.pathList)
            # 修正
            # self.reformPath()
            time.sleep(1)
        else:
            self.pathList = [[0, 0, 1, math.pi/2],
                            [1, 0, 1, math.pi/2],
                            [4.5, 0, 1, math.pi/2],
                            [4.5, -5, 1, 0],
                            [-5, -5, 1, -math.pi/2],
                            [-5, 0, 1, math.pi],
                            [0, 0, 1, math.pi/2],
                            [1, 0, 2, math.pi/2],
                            [4.5, 0,2, math.pi/2],
                            [4.5, -5, 2, 0],
                            [-5, -5, 2, -math.pi/2],
                            [-5, 0, 2, math.pi],
                            [0, 0, 2, math.pi/2]]
            self.pathList = np.array(self.pathList)
        self.landingPosition = [0, 0, 1, 0]
        self.lastUpdateTime = rospy.Time.now()
        self.cur_position = Point()
        self.targetPoint = self.pathList[0]
        self.island = False
        

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_cb)
        self.waypoint_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size=10)
        rospy.Subscriber("/planning/request_traget", Empty, self.sendTargetPoint)

    def matrix_to_euler_rad(self, matrix):
        q = tf.transformations.quaternion_from_matrix(matrix)
        eulers = tf.transformations.euler_from_quaternion(q, axes='sxyz')
        return eulers

    def reformPath(self):
        """
        进行修正，修正导航点和航向，最后一个是着陆的导航点，可以选择不进行变换
        """
        # 修正导航点
        T = self.readTMatrix()
        R = T[0:3,0:3]
        t = T[0:3, 3:4]
        loc = self.pathList[:-1, 0:3].T
        loc = np.dot(R.T, (loc-t))
        self.pathList[:-1, 0:3] = loc.T
        # 修正航向
        eulers = self.matrix_to_euler_rad(T)
        self.pathList[:, 3:4] -= eulers[2]
        print("path after fix: ")
        print(self.pathList)
    
    def readTMatrix(self):
        """
        读取初始姿态, 用于pathList的修正
        """
        rp = RosPack()
        path = rp.get_path("px4_uwb_lidar") + "/params/Transformation_maxtix.yaml"
        # 数据可能有多个空格分割，不能直接用np的库函数
        file = open(path, "r")
        lines = file.readlines()
        data = np.zeros([4, 4], np.float)
        cnt = 0
        for each in lines:
            i = 0
            while each[i]==' ':
                i += 1
            each = each[i:].replace('\n', '')
            nums = np.array(re.split(r' *', each))
            data[cnt] = nums
            cnt += 1
        file.close()
        print("变换矩阵：")
        print(data)
        return data
    
    def reachGoal(self):
        """
        判断飞机当前位置和目标位置差是否都小于阈值
        """
        return abs(self.cur_position.x-self.targetPoint[0])<self.th and \
        abs(self.cur_position.y-self.targetPoint[1])<self.th and \
        abs(self.cur_position.z-self.targetPoint[2])<self.th

    def position_cb(self, msg):
        """
        判断是否达到导航点，达到则加一
        """
        self.cur_position =  msg.pose.position
        if  self.i<len(self.pathList)-1 and self.reachGoal():
            print("add!!!!!!!!!!!!!!!!", self.i)
            self.i += 1
            self.targetPoint = self.pathList[self.i]

    def sendTargetPoint(self, msg):
        """
            发送导航点及航向信息
            接受fast-planner的topic，回调
            可以修改为service
        """
        print("tagert point: " + str(self.targetPoint))
        if self.targetPoint == []:
            return 1
        time_now = rospy.Time.now()
        init_path = Path()
        # 到达最后一个导航点
        if  self.reachGoal() and self.i==len(self.pathList)-1:
            if self.island:
                self.state = LAND
                self.landing()
            else:
                self.targetPoint = self.landingPosition
                self.island = True
            return 
                
        
        init_path.header.stamp = time_now
        init_path.header.frame_id = "world"

        current_point = PoseStamped()
        current_point.header.frame_id = "camera_init"
        current_point.header.stamp = time_now
        current_point.pose.position.x = self.targetPoint[0]
        current_point.pose.position.y = self.targetPoint[1]
        current_point.pose.position.z = self.targetPoint[2]
        #角度
        # self.targetPoint[3] = 0
        orientation = quaternion_from_euler(0, 0, self.targetPoint[3])
        current_point.pose.orientation.x = orientation[0]
        current_point.pose.orientation.y = orientation[1]
        current_point.pose.orientation.z = orientation[2]
        current_point.pose.orientation.w = orientation[3]
        init_path.poses.append(current_point)
        self.waypoint_pub.publish(init_path)
    
    def landing(self):
        """
        返航着陆
        """
        print("LAND")
        rospy.wait_for_service('land')
        try:
            # 制作服务的handle
            land_srv = rospy.ServiceProxy('land', SetBool)
            #调用服务
            resp = land_srv.call()
        except(rospy.ServiceException, e):
            print("Service call failed: %s"%e)
        
    

if __name__=="__main__":
    rospy.init_node("sendPath")
    sender = sendPath()
    rate = rospy.Rate(0.4) # 发布消息的频率
    # while not rospy.is_shutdown():
    #     # state = sender.sendTargetPoint()
    #     if(sender.state==LAND):
    #         sender.landing()
    #         break
    #     rate.sleep()
    
    rospy.spin()
