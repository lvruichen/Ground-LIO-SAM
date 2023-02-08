#! /usr/bin/python
# 解析rosbag文件，输出轨迹的tum格式数据集
import os
import rospy
import rosbag
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np


bag = rosbag.Bag('/home/eric/city_ground_res.bag')
targetFileName = '/city_gt_traj.txt'
current_directory = os.path.dirname(os.path.abspath(__file__))
fisrtMsgFlag = False
firstodomMsg = Odometry()
r1 = R.from_quat([0, 0, 0, 1])
with open(current_directory + targetFileName,'w') as f:
    # topic_list = ['/lio_sam/mapping/odometry']
    topic_list = ['/odom_basefootprint']
    for topic, msg, t in bag.read_messages(topics=topic_list):
        if(not fisrtMsgFlag):
            fisrtMsgFlag = True
            firstodomMsg = msg
            r1 = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            print(firstodomMsg.pose.pose.position.x)
            print(firstodomMsg.pose.pose.position.y)
            print(firstodomMsg.pose.pose.position.z)
            print(r1.as_matrix())
            continue
        timeStamp = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        x = x - firstodomMsg.pose.pose.position.x
        y = msg.pose.pose.position.y
        y = y - firstodomMsg.pose.pose.position.y
        z = msg.pose.pose.position.z
        z = z - firstodomMsg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        r2 = R.from_quat([qx, qy, qz, qw])
        r3 = r1.inv() * r2
        f.write(str(timeStamp) + ' ' + str(y) + ' ' + str(-x) + ' ' + str(z) + ' ' 
        + str(r3.as_quat()[1]) + ' ' + str(r3.as_quat()[0]) + ' ' + str(r3.as_quat()[2]) + ' ' + str(r3.as_quat()[3]) + '\n')
    bag.close()
