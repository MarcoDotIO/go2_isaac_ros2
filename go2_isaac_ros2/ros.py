import rclpy
from rclpy.node import Node
import threading
import torch

from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowCmd, LowState
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from go2_isaac_ros2.env import IsaacSimGo2EnvWrapper
from go2_isaac_ros2.lidar import get_head_lidar_pointcloud


GROUND_TRUTH_TWIST_TOPIC = "/ground_truth/twist"
GROUND_TRUTH_WORLD_POSE_TOPIC = "/vrpn_mocap/go2/pose"
GROUND_TRUTH_WORLD_TWIST_TOPIC = "/vrpn_mocap/go2/twist"
WORLD_FRAME_ID = "world"
BASE_FRAME_ID = "base"
HEAD_LIDAR_FRAME_ID = "utlidar_lidar"
FRONT_CAMERA_FRAME_ID = "front_cam"
HEAD_LIDAR_POS = (0.0, 0.0, 0.06)
FRONT_CAMERA_POS = (0.32487, -0.00095, 0.05362)
FRONT_CAMERA_QUAT_WXYZ = (0.5, -0.5, 0.5, -0.5)


class Go2SubNode(Node):
    def __init__(self, env: IsaacSimGo2EnvWrapper):
        super().__init__("go2_sub_node")

        self.lowcmd_sub = self.create_subscription(LowCmd, "/lowcmd", self.lowcmd_cb, 1)
        self.thead = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.env = env

    def lowcmd_cb(self, msg: LowCmd):
        action = torch.zeros(12)
        stiffness = torch.zeros(12)
        damping = torch.zeros(12)
        for i in range(12):
            action[i] = msg.motor_cmd[i].q
            stiffness[i] = msg.motor_cmd[i].kp
            damping[i] = msg.motor_cmd[i].kd
        self.env.set_action(action)
        self.env.set_stiffness(stiffness)
        self.env.set_damping(damping)

    def start(self):
        self.thead.start()


class Go2PubNode(Node):
    def __init__(self):
        super().__init__("go2_pub_node")

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.low_state_pub = self.create_publisher(LowState, "/lowstate", 10)
        self.head_lidar_pub = self.create_publisher(PointCloud2, "utlidar/cloud", 10)
        self.ground_truth_twist_pub = self.create_publisher(
            Twist, GROUND_TRUTH_TWIST_TOPIC, 10
        )
        self.ground_truth_world_pose_pub = self.create_publisher(
            PoseStamped, GROUND_TRUTH_WORLD_POSE_TOPIC, 10
        )
        self.ground_truth_world_twist_pub = self.create_publisher(
            TwistStamped, GROUND_TRUTH_WORLD_TWIST_TOPIC, 10
        )
        self.clock_msg = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._pub_static_tf()

    def publish(self, obs: dict, sim_time_sec: float):
        self._pub_clock(sim_time_sec)
        self._pub_tf(obs)
        self._pub_low_state(obs)
        self._pub_head_lidar()
        self._pub_ground_truth(obs)

    def _pub_clock(self, sim_time_sec: float):
        msg = Clock()
        msg.clock = self.get_clock().now().to_msg()
        msg.clock.sec = int(sim_time_sec)
        msg.clock.nanosec = int((sim_time_sec - int(sim_time_sec)) * 1e9)
        self.clock_msg = msg
        self.clock_pub.publish(msg)

    def _pub_low_state(self, obs: dict):
        msg = LowState()

        for i in range(12):
            msg.motor_state[i].q = obs["obs"]["joint_pos"][0, i].item()
            msg.motor_state[i].dq = obs["obs"]["joint_vel"][0, i].item()

        msg.imu_state.quaternion[0] = obs["obs"]["imu_body_orientation"][0, 0].item()
        msg.imu_state.quaternion[1] = obs["obs"]["imu_body_orientation"][0, 1].item()
        msg.imu_state.quaternion[2] = obs["obs"]["imu_body_orientation"][0, 2].item()
        msg.imu_state.quaternion[3] = obs["obs"]["imu_body_orientation"][0, 3].item()
        msg.imu_state.gyroscope[0] = obs["obs"]["imu_body_ang_vel"][0, 0].item()
        msg.imu_state.gyroscope[1] = obs["obs"]["imu_body_ang_vel"][0, 1].item()
        msg.imu_state.gyroscope[2] = obs["obs"]["imu_body_ang_vel"][0, 2].item()
        msg.imu_state.accelerometer[0] = obs["obs"]["imu_body_lin_acc"][0, 0].item()
        msg.imu_state.accelerometer[1] = obs["obs"]["imu_body_lin_acc"][0, 1].item()
        msg.imu_state.accelerometer[2] = obs["obs"]["imu_body_lin_acc"][0, 2].item()

        self.low_state_pub.publish(msg)

    def _pub_head_lidar(self):
        pcl = get_head_lidar_pointcloud().tolist()

        header = Header()
        header.frame_id = HEAD_LIDAR_FRAME_ID
        header.stamp.sec = self.clock_msg.clock.sec
        header.stamp.nanosec = self.clock_msg.clock.nanosec

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=16, datatype=PointField.FLOAT32, count=1
            ),
            PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=24, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_msg = point_cloud2.create_cloud(header, fields, pcl)
        self.head_lidar_pub.publish(pcl_msg)

    def _pub_ground_truth(self, obs: dict):
        twist = Twist()
        twist.linear.x = obs["obs"]["body_lin_vel"][0, 0].item()
        twist.linear.y = obs["obs"]["body_lin_vel"][0, 1].item()
        twist.linear.z = obs["obs"]["body_lin_vel"][0, 2].item()
        twist.angular.x = obs["obs"]["body_ang_vel"][0, 0].item()
        twist.angular.y = obs["obs"]["body_ang_vel"][0, 1].item()
        twist.angular.z = obs["obs"]["body_ang_vel"][0, 2].item()
        self.ground_truth_twist_pub.publish(twist)

        pose_w = PoseStamped()
        pose_w.header.frame_id = WORLD_FRAME_ID
        pose_w.header.stamp.sec = self.clock_msg.clock.sec
        pose_w.header.stamp.nanosec = self.clock_msg.clock.nanosec
        pose_w.pose.position.x = obs["obs"]["world_pos"][0, 0].item()
        pose_w.pose.position.y = obs["obs"]["world_pos"][0, 1].item()
        pose_w.pose.position.z = obs["obs"]["world_pos"][0, 2].item()
        pose_w.pose.orientation.x = obs["obs"]["world_quat"][0, 1].item()
        pose_w.pose.orientation.y = obs["obs"]["world_quat"][0, 2].item()
        pose_w.pose.orientation.z = obs["obs"]["world_quat"][0, 3].item()
        pose_w.pose.orientation.w = obs["obs"]["world_quat"][0, 0].item()
        self.ground_truth_world_pose_pub.publish(pose_w)

        twist_w = TwistStamped()
        twist_w.header.frame_id = WORLD_FRAME_ID
        twist_w.header.stamp.sec = self.clock_msg.clock.sec
        twist_w.header.stamp.nanosec = self.clock_msg.clock.nanosec
        twist_w.twist.linear.x = obs["obs"]["world_lin_vel"][0, 0].item()
        twist_w.twist.linear.y = obs["obs"]["world_lin_vel"][0, 1].item()
        twist_w.twist.linear.z = obs["obs"]["world_lin_vel"][0, 2].item()
        twist_w.twist.angular.x = obs["obs"]["world_ang_vel"][0, 0].item()
        twist_w.twist.angular.y = obs["obs"]["world_ang_vel"][0, 1].item()
        twist_w.twist.angular.z = obs["obs"]["world_ang_vel"][0, 2].item()
        self.ground_truth_world_twist_pub.publish(twist_w)

    def _pub_static_tf(self):
        stamp = self.get_clock().now().to_msg()

        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = stamp
        lidar_tf.header.frame_id = BASE_FRAME_ID
        lidar_tf.child_frame_id = HEAD_LIDAR_FRAME_ID
        lidar_tf.transform.translation.x = HEAD_LIDAR_POS[0]
        lidar_tf.transform.translation.y = HEAD_LIDAR_POS[1]
        lidar_tf.transform.translation.z = HEAD_LIDAR_POS[2]
        lidar_tf.transform.rotation.w = 1.0

        front_cam_tf = TransformStamped()
        front_cam_tf.header.stamp = stamp
        front_cam_tf.header.frame_id = BASE_FRAME_ID
        front_cam_tf.child_frame_id = FRONT_CAMERA_FRAME_ID
        front_cam_tf.transform.translation.x = FRONT_CAMERA_POS[0]
        front_cam_tf.transform.translation.y = FRONT_CAMERA_POS[1]
        front_cam_tf.transform.translation.z = FRONT_CAMERA_POS[2]
        front_cam_tf.transform.rotation.x = FRONT_CAMERA_QUAT_WXYZ[1]
        front_cam_tf.transform.rotation.y = FRONT_CAMERA_QUAT_WXYZ[2]
        front_cam_tf.transform.rotation.z = FRONT_CAMERA_QUAT_WXYZ[3]
        front_cam_tf.transform.rotation.w = FRONT_CAMERA_QUAT_WXYZ[0]

        self.tf_static_broadcaster.sendTransform([lidar_tf, front_cam_tf])

    def _pub_tf(self, obs: dict):
        if self.clock_msg is None:
            return

        base_tf = TransformStamped()
        base_tf.header.stamp.sec = self.clock_msg.clock.sec
        base_tf.header.stamp.nanosec = self.clock_msg.clock.nanosec
        base_tf.header.frame_id = WORLD_FRAME_ID
        base_tf.child_frame_id = BASE_FRAME_ID
        base_tf.transform.translation.x = obs["obs"]["world_pos"][0, 0].item()
        base_tf.transform.translation.y = obs["obs"]["world_pos"][0, 1].item()
        base_tf.transform.translation.z = obs["obs"]["world_pos"][0, 2].item()
        base_tf.transform.rotation.x = obs["obs"]["world_quat"][0, 1].item()
        base_tf.transform.rotation.y = obs["obs"]["world_quat"][0, 2].item()
        base_tf.transform.rotation.z = obs["obs"]["world_quat"][0, 3].item()
        base_tf.transform.rotation.w = obs["obs"]["world_quat"][0, 0].item()
        self.tf_broadcaster.sendTransform(base_tf)

    def _clock_to_sec(self, clock_msg: Clock) -> float:
        return clock_msg.clock.sec + clock_msg.clock.nanosec / 1e9
