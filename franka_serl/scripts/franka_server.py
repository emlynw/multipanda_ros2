"""
This file starts a control server running on the real time PC connected to the franka robot.
In a screen run `python franka_server.py`
"""
from flask import Flask, request, jsonify
import numpy as np
import time
import subprocess
from scipy.spatial.transform import Rotation as R
from absl import app, flags


import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError

from franka_msgs.msg import FrankaState
from franka_msgs.srv import ErrorRecovery
import geometry_msgs.msg as geom_msg

FLAGS = flags.FLAGS
flags.DEFINE_string(
    "robot_ip", "172.16.0.2", "IP address of the franka robot's controller box"
)
flags.DEFINE_string(
    "gripper_ip", "192.168.1.114", "IP address of the robotiq gripper if being used"
)
flags.DEFINE_string(
    "gripper_type", "Franka", "Type of gripper to use: Robotiq, Franka, or None"
)



class FrankaServer(Node):
    """Handles the starting and stopping of the impedance controller
    (as well as backup) joint recovery policy."""

    def __init__(self, robot_ip, gripper_type, ros_pkg_name):
        super().__init__('franka_control_api')
        self.robot_ip = robot_ip
        self.ros_pkg_name = ros_pkg_name
        self.gripper_type = gripper_type

        # ROS Subscribers
        custom_qos_profile = qos.QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.VOLATILE,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.state_sub = self.create_subscription(
            FrankaState, 
            '/franka_robot_state_broadcaster/robot_state',
            self._set_currpos,
            qos_profile=custom_qos_profile,
            callback_group=self.callback_group
        )
        self.gripper_sub = self.create_subscription(
            JointState,
            '/panda_gripper/joint_states',
            self._set_gripper_pos,
            qos_profile=custom_qos_profile,
            callback_group=self.callback_group
        )
        ## Todo: Create jacobian publisher in cartesian_impedance_controller
        # self.jacobian_sub = self.node.create_subscription(
        #     ee_zero_jacobian,
        #     '/cartesian_impedance_controller/franka_jacobian',
        #     self._set_jacobian,
        #     qos_profile=custom_qos_profile,
        #     callback_group=self.callback_group
        # )

        self.eepub = self.create_publisher(Pose, '/franka/goal_pose', 10)
        self.gripper_pub = self.create_publisher(Float32, '/franka/gripper', 10)

        # Error Recovery Service Client
        self.error_recovery_client = self.create_client(ErrorRecovery, '/panda_error_recovery_service_server/error_recovery')


    def start_impedance(self):
        """Launches the impedance controller"""
        self.imp = subprocess.Popen(
            [
                "ros2", "launch",
                self.ros_pkg_name,
                "impedance.launch.py",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(5)

    def stop_impedance(self):
        """Stops the impedance controller"""
        self.imp.terminate()
        time.sleep(1)

    def clear(self):
        """Clears any errors"""
        if self.error_recovery_client.wait_for_service(timeout_sec=5.0):
            request = ErrorRecovery.Request()
            future = self.error_recovery_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info("Successfully cleared errors")
            else:
                self.get_logger().error("Failed to clear errors")
        else:
            self.get_logger().error('Error recovery service not available')

    def reset_joint(self):
        """Resets Joints (needed after running for hours)"""
        # First Stop impedance
        try:
            self.stop_impedance()
            self.clear()
        except:
            print("impedance Not Running")
        time.sleep(3)
        self.clear()

        # Launch joint controller reset
        # set rosparm with rospkg
        # rosparam set /target_joint_positions '[q1, q2, q3, q4, q5, q6, q7]'
        # rospy.set_param("/target_joint_positions", self.reset_joint_target)
        # Todo: Make target positions adjustable

        self.joint_controller = subprocess.Popen(
            [
                "ros2", "launch",
                self.ros_pkg_name,
                "reset.launch.py",
                "robot_ip:=" + self.robot_ip,
                f"load_gripper:={'true' if self.gripper_type == 'Franka' else 'false'}",
            ],
            stdout=subprocess.PIPE,
        )
        time.sleep(1)
        print("RUNNING JOINT RESET")
        self.clear()

        # Wait until target joint angles are reached
        count = 0
        time.sleep(1)
        while not np.allclose(
            np.array(self.reset_joint_target) - np.array(self.q),
            0,
            atol=1e-2,
            rtol=1e-2,
        ):
            time.sleep(1)
            count += 1
            if count > 30:
                print("joint reset TIMEOUT")
                break

        # Stop joint controller
        print("RESET DONE")
        self.joint_controller.terminate()
        time.sleep(1)
        self.clear()
        print("KILLED JOINT RESET", self.pos)

        # Restart impedece controller
        self.start_impedance()
        print("impedance STARTED")

    def move(self, pose: list):
        """Moves to a pose: [x, y, z, qx, qy, qz, qw]"""
        assert len(pose) == 7
        msg = Pose()
        msg.position = geom_msg.Point(pose[0], pose[1], pose[2])
        msg.orientation = geom_msg.Quaternion(pose[3], pose[4], pose[5], pose[6])
        self.eepub.publish(msg)

    def open(self):
        msg = Float32()
        msg.data = -1.0
        self.gripper_pub.publish(msg)

    def close(self):
        msg = Float32()
        msg.data = 1.0
        self.gripper_pub.publish(msg)

    def _set_currpos(self, msg):
        tmatrix = np.array(list(msg.o_t_ee)).reshape(4, 4).T
        r = R.from_matrix(tmatrix[:3, :3])
        pose = np.concatenate([tmatrix[:3, -1], r.as_quat()])
        self.pos = pose
        self.dq = np.array(list(msg.dq)).reshape((7,))
        self.q = np.array(list(msg.q)).reshape((7,))
        self.force = np.array(list(msg.k_f_ext_hat_k)[:3])
        self.torque = np.array(list(msg.k_f_ext_hat_k)[3:])
        # try:
        #     self.vel = self.jacobian @ self.dq
        # except:
        #     self.vel = np.zeros(6)

    def _set_gripper_pos(self, msg):
        self.gripper_pos = np.float32(msg.position[0])

    # def _set_jacobian(self, msg):
    #     jacobian = np.array(list(msg.zero_jacobian)).reshape((6, 7), order="F")
    #     self.jacobian = jacobian


###############################################################################


def main(_):
    rclpy.init(args=args)


    ROS_PKG_NAME = "franka_serl"

    ROBOT_IP = FLAGS.robot_ip
    GRIPPER_IP = FLAGS.gripper_ip
    GRIPPER_TYPE = FLAGS.gripper_type

    webapp = Flask(__name__)

    robot_server = FrankaServer(
        robot_ip=ROBOT_IP,
        gripper_type=GRIPPER_TYPE,
        ros_pkg_name=ROS_PKG_NAME,
    )
    rclpy.spin(robot_server)

    robot_server.start_impedance()

    # Route for Starting impedance
    @webapp.route("/startimp", methods=["POST"])
    def start_impedance():
        robot_server.clear()
        robot_server.start_impedance()
        return "Started impedance"

    # Route for Stopping impedance
    @webapp.route("/stopimp", methods=["POST"])
    def stop_impedance():
        robot_server.stop_impedance()
        return "Stopped impedance"

    # Route for Getting Pose
    @webapp.route("/getpos", methods=["POST"])
    def get_pos():
        return jsonify({"pose": np.array(robot_server.pos).tolist()})

    @webapp.route("/getpos_euler", methods=["POST"])
    def get_pos_euler():
        r = R.from_quat(robot_server.pos[3:])
        euler = r.as_euler("xyz")
        return jsonify({"pose": np.concatenate([robot_server.pos[:3], euler]).tolist()})

    # @webapp.route("/getvel", methods=["POST"])
    # def get_vel():
    #     return jsonify({"vel": np.array(robot_server.vel).tolist()})

    @webapp.route("/getforce", methods=["POST"])
    def get_force():
        return jsonify({"force": np.array(robot_server.force).tolist()})

    @webapp.route("/gettorque", methods=["POST"])
    def get_torque():
        return jsonify({"torque": np.array(robot_server.torque).tolist()})

    @webapp.route("/getq", methods=["POST"])
    def get_q():
        return jsonify({"q": np.array(robot_server.q).tolist()})

    @webapp.route("/getdq", methods=["POST"])
    def get_dq():
        return jsonify({"dq": np.array(robot_server.dq).tolist()})

    # @webapp.route("/getjacobian", methods=["POST"])
    # def get_jacobian():
    #     return jsonify({"jacobian": np.array(robot_server.jacobian).tolist()})

    # Route for getting gripper distance
    @webapp.route("/get_gripper", methods=["POST"])
    def get_gripper():
        return jsonify({"gripper": gripper_server.gripper_pos})

    # Route for Running Joint Reset
    @webapp.route("/jointreset", methods=["POST"])
    def joint_reset():
        robot_server.clear()
        robot_server.reset_joint()
        return "Reset Joint"

    # # Route for Activating the Gripper
    # @webapp.route("/activate_gripper", methods=["POST"])
    # def activate_gripper():
    #     print("activate gripper")
    #     gripper_server.activate_gripper()
    #     return "Activated"

    # # Route for Resetting the Gripper. It will reset and activate the gripper
    # @webapp.route("/reset_gripper", methods=["POST"])
    # def reset_gripper():
    #     print("reset gripper")
    #     gripper_server.reset_gripper()
    #     return "Reset"

    # Route for Opening the Gripper
    @webapp.route("/open_gripper", methods=["POST"])
    def open():
        print("open")
        gripper_server.open()
        return "Opened"

    # Route for Closing the Gripper
    @webapp.route("/close_gripper", methods=["POST"])
    def close():
        print("close")
        gripper_server.close()
        return "Closed"

    # # Route for moving the gripper
    # @webapp.route("/move_gripper", methods=["POST"])
    # def move_gripper():
    #     gripper_pos = request.json
    #     pos = np.clip(int(gripper_pos["gripper_pos"]), 0, 255)  # 0-255
    #     print(f"move gripper to {pos}")
    #     gripper_server.move(pos)
    #     return "Moved Gripper"

    # Route for Clearing Errors (Communcation constraints, etc.)
    @webapp.route("/clearerr", methods=["POST"])
    def clear():
        robot_server.clear()
        return "Clear"

    # Route for Sending a pose command
    @webapp.route("/pose", methods=["POST"])
    def pose():
        pos = np.array(request.json["arr"])
        print("Moving to", pos)
        robot_server.move(pos)
        return "Moved"

    # Route for getting all state information
    @webapp.route("/getstate", methods=["POST"])
    def get_state():
        return jsonify(
            {
                "pose": np.array(robot_server.pos).tolist(),
                # "vel": np.array(robot_server.vel).tolist(),
                "force": np.array(robot_server.force).tolist(),
                "torque": np.array(robot_server.torque).tolist(),
                "q": np.array(robot_server.q).tolist(),
                "dq": np.array(robot_server.dq).tolist(),
                # "jacobian": np.array(robot_server.jacobian).tolist(),
                "gripper_pos": gripper_server.gripper_pos,
            }
        )

    # # Route for updating compliance parameters
    # @webapp.route("/update_param", methods=["POST"])
    # def update_param():
    #     reconf_client.update_configuration(request.json)
    #     return "Updated compliance parameters"

    webapp.run(host="0.0.0.0")


if __name__ == "__main__":
    app.run(main)