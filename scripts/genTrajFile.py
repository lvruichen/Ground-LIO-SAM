#! /usr/bin/python
# 解析rosbag文件，输出轨迹的tum格式数据集
# 需要修改的是rosbag的路径以及需要处理的消息名

import os
import rospy
import rosbag
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
from numpy.linalg import inv,pinv

bag_path = '/home/eric/data/simulation_demo_dynamic/demo_dyna_4_filter_res.bag'
# topic_list = ['/odom_basefootprint']
topic_list = ['/lio_sam/mapping/odometry']


bag = rosbag.Bag(bag_path)
if topic_list[0] == '/odom_basefootprint':
    targetFileName = bag_path[0:len(bag_path) - 4] + '_gt.txt'
else:
    targetFileName = bag_path[0:len(bag_path) - 4] + '.txt'


current_directory = os.path.dirname(os.path.abspath(__file__))
fisrtMsgFlag = False
firstodomMsg = Odometry()
firstTimeStamp = 0
firstTransmatrix = np.eye(4)
R_0 = R.from_quat([0, 0, 0, 1])
T_lidar2basefootprint = np.eye(4)
r_l2b = R.from_quat([0, 0, 0.707, 0.707])
# t_l2b = [0.233, 0.000, 1.212]
T_lidar2basefootprint[:3, :3] = r_l2b.as_matrix()
# T_lidar2basefootprint[:3, 3] = t_l2b

with open(targetFileName,'w') as f:
   
    
    for topic, msg, t in bag.read_messages(topics=topic_list):
        if(not fisrtMsgFlag):
            fisrtMsgFlag = True
            firstodomMsg = msg
            R_0 = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            firstTransmatrix[:3, :3] = R_0.as_matrix()
            t0 = np.array([firstodomMsg.pose.pose.position.x, firstodomMsg.pose.pose.position.y, firstodomMsg.pose.pose.position.z])
            firstTransmatrix[:3, 3] = t0
            firstTimeStamp = msg.header.stamp.to_sec()
            print(firstTransmatrix)
            continue
        timeStamp = msg.header.stamp.to_sec()
        timeStamp -= firstTimeStamp
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        t1 = np.array([x, y, z])
        R_1 = R.from_quat([qx, qy, qz, qw])
        curTransmatrix = np.eye(4)
        curTransmatrix[:3, :3] = R_1.as_matrix()
        curTransmatrix[:3, 3] = t1
        increMatrix = inv(firstTransmatrix).dot(curTransmatrix)
        R_incre = R.from_matrix(increMatrix[:3, :3])
        x_ = increMatrix[0][3]
        y_ = increMatrix[1][3]
        z_ = increMatrix[2][3]
        qx_ = R_incre.as_quat()[0]
        qy_ = R_incre.as_quat()[1]
        qz_ = R_incre.as_quat()[2]
        qw_ = R_incre.as_quat()[3]
        if(topic_list[0] == '/lio_sam/mapping/odometry'):
            f.write(str(timeStamp) + ' ' + str(-y_) + ' ' + str(x_) + ' ' + str(z_) + ' ' 
            + str(-qy_) + ' ' + str(qx_) + ' ' + str(qz_) + ' ' + str(qw_) + '\n')
        else:
            f.write(str(timeStamp) + ' ' + str(x_) + ' ' + str(y_) + ' ' + str(z_) + ' ' 
            + str(qx_) + ' ' + str(qy_) + ' ' + str(qz_) + ' ' + str(qw_) + '\n') 
    bag.close()
