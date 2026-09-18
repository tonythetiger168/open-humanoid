#!/usr/bin/env python3
"""OpenHumanoid v1.0 - ROS2 Control Node

Publishes:
    /joint_states (sensor_msgs/JointState)
    /tf (geometry_msgs/TransformStamped)
    /imu/data (sensor_msgs/Imu)
    /foot_force (geometry_msgs/WrenchStamped)

Subscribes:
    /cmd_vel (geometry_msgs/Twist) - teleoperation commands
    /cmd_mode (std_msgs/String) - mode switch (stand/walk/teleop)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped, WrenchStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import numpy as np
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from simulation.pybullet_env import OpenHumanoidSim, SimConfig
from control.main_controller import RobotController, ControlMode

class HumanoidNode(Node):
    def __init__(self):
        super().__init__("open_humanoid_control")

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.lf_pub = self.create_publisher(WrenchStamped, "foot/left", 10)
        self.rf_pub = self.create_publisher(WrenchStamped, "foot/right", 10)

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(String, "cmd_mode", self.cmd_mode_cb, 10)

        # Simulation + Controller
        self.cfg = SimConfig(
            urdf_path="design/open_humanoid_v1.urdf",
            use_gui=False,
            sim_freq=240.0,
            control_freq=50.0
        )
        self.sim = OpenHumanoidSim(self.cfg)
        self.sim.init()
        self.controller = RobotController(self.sim, mode=ControlMode.STAND)
        self.controller.reset()

        # Timer
        self.dt = 1.0 / self.cfg.control_freq
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("OpenHumanoid ROS2 node initialized")

    def cmd_vel_cb(self, msg):
        self.controller.set_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def cmd_mode_cb(self, msg):
        mode_map = {"stand": ControlMode.STAND, "walk": ControlMode.WALK, "teleop": ControlMode.TELEOP}
        if msg.data in mode_map:
            self.controller.set_mode(mode_map[msg.data])

    def control_loop(self):
        # Step simulation
        self.sim.step(controller_callback=lambda s: self.controller.update())

        # Publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.sim.joint_states.keys())
        js.position = [self.sim.joint_states[n]["position"] for n in js.name]
        js.velocity = [self.sim.joint_states[n]["velocity"] for n in js.name]
        self.joint_pub.publish(js)

        # Publish IMU
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"
        imu.angular_velocity.x = self.sim.imu_data["gyro"][0]
        imu.angular_velocity.y = self.sim.imu_data["gyro"][1]
        imu.angular_velocity.z = self.sim.imu_data["gyro"][2]
        imu.orientation.x = self.sim.imu_data["quat"][0]
        imu.orientation.y = self.sim.imu_data["quat"][1]
        imu.orientation.z = self.sim.imu_data["quat"][2]
        imu.orientation.w = self.sim.imu_data["quat"][3]
        self.imu_pub.publish(imu)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.sim.base_pose["pos"][0]
        t.transform.translation.y = self.sim.base_pose["pos"][1]
        t.transform.translation.z = self.sim.base_pose["pos"][2]
        t.transform.rotation.x = self.sim.base_pose["orn"][0]
        t.transform.rotation.y = self.sim.base_pose["orn"][1]
        t.transform.rotation.z = self.sim.base_pose["orn"][2]
        t.transform.rotation.w = self.sim.base_pose["orn"][3]
        self.tf_broadcaster.sendTransform(t)

        # Publish foot forces
        for foot, pub in [("left", self.lf_pub), ("right", self.rf_pub)]:
            w = WrenchStamped()
            w.header.stamp = self.get_clock().now().to_msg()
            w.header.frame_id = f"{foot}_foot_link"
            w.wrench.force.z = self.sim.foot_force[foot]
            pub.publish(w)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sim.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
