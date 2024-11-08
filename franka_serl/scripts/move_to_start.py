# Python libs
import rclpy
import time
from rclpy.node import Node
from rclpy import qos
# OpenCV
import cv2
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

# ROS libraries
import image_geometry

# ROS Messages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from franka_msgs.msg import FrankaState

def ImpedenceTest():
    # camera_model = None

        rclpy.init()
        node = rclpy.create_node('franka_publisher')
    
        work_dir = Path.cwd()
        bridge = CvBridge()
        
        
        object_location_pub = node.create_publisher(Pose, '/franka/goal_pose', 10)

        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        print(pose)
        for i in range(10):
            object_location_pub.publish(pose) 
            time.sleep(0.2)
        
       

def main(args=None):
    ImpedenceTest()

if __name__ == '__main__':
    main()