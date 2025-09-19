#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

M = 75  # obstacle threshold
N = 50  # free threshold

pub = None

def callback(cmap):
    data = np.array(cmap.data, dtype=np.int16)  # 转 numpy
    # 分类映射
    data[data >= M] = 100
    data[(data >= 0) & (data < N)] = 0
    data[(data < 0) | (data >= N) & (data < M)] = -1
    # 更新消息
    cmap.data = tuple(data.tolist())
    pub.publish(cmap)

if __name__ == "__main__":
    rospy.init_node("mapc_node", anonymous=True)
    pub = rospy.Publisher("/cmap", OccupancyGrid, queue_size=20)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.spin()
