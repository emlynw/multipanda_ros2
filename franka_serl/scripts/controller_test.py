import rclpy
import time
from rclpy.node import Node
from rclpy import qos
import numpy as np

# ROS libraries

# ROS Messages
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from franka_msgs.msg import FrankaState

class ControllerTest(Node):

    def __init__(self):
        super().__init__('controller_test')

        self.state_sub = self.create_subscription(FrankaState, '/franka_robot_state_broadcaster/robot_state', 
                                                  self.state_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.goal_pose_pub = self.create_publisher(Pose, '/franka/goal_pose', 10)
        self.gripper_pub = self.create_publisher(Float32, '/franka/gripper', 10)
        timer = self.create_timer(0.1, self.timer_callback)
        self.state_initialized = False
        self._CARTESIAN_BOUNDS = np.asarray([[0.2, -0.4, 0], [0.8, 0.4, 0.6]]) 
        self.action_scale = 0.1 
        self.i=0

        self.init_pose = Pose()
        self.init_pose.position.x = 0.3
        self.init_pose.position.y = 0.0
        self.init_pose.position.z = 0.5
        self.init_pose.orientation.x = 1.0
        self.init_pose.orientation.y = 0.0
        self.init_pose.orientation.z = 0.0
        self.init_pose.orientation.w = 0.0
        self.goal_pose_pub.publish(self.init_pose)
        time.sleep(5)

    def state_callback(self, data):
        self.x = data.o_t_ee[12]
        self.y = data.o_t_ee[13]
        self.z = data.o_t_ee[14]
        self.state_initialized = True

    def reset(self):
        pose = Pose()
        pose.position.x = self.init_pose.position.x
        pose.position.y = self.init_pose.position.y
        pose.position.z = self.init_pose.position.z
        pose.orientation.x = self.init_pose.orientation.x
        pose.orientation.y = self.init_pose.orientation.y
        pose.orientation.z = self.init_pose.orientation.z
        pose.orientation.w = self.init_pose.orientation.w
        for i in range(10):
            self.goal_pose_pub.publish(pose) 
            self.gripper_pub.publish(Float32(data=-1.0))
            time.sleep(0.1)
        time.sleep(4)

    def step(self, action):
        pose = Pose()
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.x = self.x + self.action_scale*action[0]
        pose.position.y = self.y + self.action_scale*action[1]
        pose.position.z = self.z + self.action_scale*action[2]
        # Clip the position to the bounds
        pose.position.x = np.clip(pose.position.x, self._CARTESIAN_BOUNDS[0, 0], self._CARTESIAN_BOUNDS[1, 0])
        pose.position.y = np.clip(pose.position.y, self._CARTESIAN_BOUNDS[0, 1], self._CARTESIAN_BOUNDS[1, 1])
        pose.position.z = np.clip(pose.position.z, self._CARTESIAN_BOUNDS[0, 2], self._CARTESIAN_BOUNDS[1, 2])
        self.goal_pose_pub.publish(pose) 
        self.gripper_pub.publish(Float32(data=action[3]))
        return
    
    def timer_callback(self):
        if not self.state_initialized:
            return
        if self.i < 20:
            action = np.array([0.0, -0.5, 0.0, -1.0])
            self.step(action)
        elif self.i < 40:
            action = np.array([0.0, 0.0, -0.5, 1.0])
            self.step(action)
        elif self.i < 60:
            action = np.array([0.5, 0.0, 0.0, -1.0])
            self.step(action)
        elif self.i < 80:
            action = np.array([0.0, 0.5, 0.0, 1.0])
            self.step(action)
        else:
            self.i = 0
            self.reset()
            return
        self.i+=1


def main(args=None):
    rclpy.init(args=args)
    controller_test = ControllerTest()
    rclpy.spin(controller_test)
    controller_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()