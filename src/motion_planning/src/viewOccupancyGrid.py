#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Length of data is " + str(len(data.data)))
    
    image = np.zeros((data.info.height, data.info.width))
    print(data.data[0])

    index = 0
    for i in range(data.info.height):
        for j in range(data.info.width):
            
            image[i][j] = data.data[index]
            index = index + 1

    plt.imshow(image)
    plt.show() 


def showOccupancyGrid():
    rospy.init_node('viewOccupancyGrids', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    showOccupancyGrid()
