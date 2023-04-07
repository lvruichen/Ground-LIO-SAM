#!/usr/bin/bash

cd /home/eric/data/binhai/slam
rosbag record /lidar_points /lio_sam/deskew/cloud_deskewed /odometry/imu_incremental /clock -O binhhai_lio_res.bag