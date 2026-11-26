"""ROS 2 side of the excavator test: OPERA's ZX200 arm on ros2_control mock hardware with a real
JointTrajectoryController, an odometry publisher for the parked machine, and a joint-state recorder.

usage: python excavator_rig.py URDF_IN OUT_DIR X Y Z YAW     (run inside a ROS 2 Jazzy environment)
Writes OUT_DIR/joints.jsonl; stops on SIGTERM.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

URDF_IN, OUT, X, Y, Z, YAW = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7])
# Where the arm starts (radians).
INIT = {"swing_joint": 0.0, "boom_joint": -0.8, "arm_joint": 1.5, "bucket_joint": 1.0,  # bucket up, 2.6 m
        "bucket_end_joint": 0.0}
NS = "zx200"

with open(URDF_IN, encoding="utf-8") as f:
    urdf = f.read()
# Add a ros2_control section to the URDF: mock hardware that follows position commands exactly, in place of the real
# machine's hydraulics.
block = "".join(f"""<joint name="{j}"><command_interface name="position"/>
  <state_interface name="position"><param name="initial_value">{v}</param></state_interface>
  <state_interface name="velocity"/></joint>""" for j, v in INIT.items())
urdf = urdf.replace("</robot>", f"""<ros2_control name="zx200" type="system"><hardware>
  <plugin>mock_components/GenericSystem</plugin></hardware>{block}</ros2_control></robot>""")
os.makedirs(OUT, exist_ok=True)
with open(f"{OUT}/zx200_mock.urdf", "w", encoding="utf-8") as f:
    f.write(urdf)
with open(f"{OUT}/controllers.yaml", "w", encoding="utf-8") as f:
    f.write("""/**:
  controller_manager:
    ros__parameters:
      update_rate: 100
      upper_arm_controller: {type: joint_trajectory_controller/JointTrajectoryController}
      joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
  upper_arm_controller:
    ros__parameters:
      joints: [swing_joint, boom_joint, arm_joint, bucket_joint, bucket_end_joint]
      command_interfaces: [position]
      state_interfaces: [position, velocity]
""")
ns = ["--ros-args", "-r", f"__ns:=/{NS}"]
# Start the robot state publisher and the controller manager, then load the two controllers.
procs = [
    subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", *ns,
                      "-p", f"robot_description:={urdf}"]),
    subprocess.Popen(["ros2", "run", "controller_manager", "ros2_control_node", *ns,
                      "--params-file", f"{OUT}/controllers.yaml"]),
]
time.sleep(3)
for c in ("joint_state_broadcaster", "upper_arm_controller"):
    subprocess.run(["ros2", "run", "controller_manager", "spawner", c, "-c", f"/{NS}/controller_manager",
                    "--param-file", f"{OUT}/controllers.yaml", *ns], check=True, timeout=60)

rclpy.init()
node = rclpy.create_node("rig")
odom_pub = node.create_publisher(Odometry, f"/{NS}/odom", 10)
log = open(f"{OUT}/joints.jsonl", "w", encoding="utf-8")  # noqa: SIM115
node.create_subscription(JointState, f"/{NS}/joint_states",
                         lambda m: log.write(json.dumps({"t": time.time(), **dict(zip(
                             m.name, m.position, strict=False))}) + "\n"), 100)


def odom():
    """Publish the parked machine's fixed position and heading."""
    m = Odometry()
    m.header.frame_id = "map"
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = X, Y, Z
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(YAW / 2), math.cos(YAW / 2)
    odom_pub.publish(m)


node.create_timer(0.05, odom)
running = True
# SIGTERM ends the loop below, so the log is closed and the ROS processes are shut down in order.
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("running", False))
print("RIG READY", flush=True)
while running:
    rclpy.spin_once(node, timeout_sec=0.05)
log.close()
for p in procs:
    p.send_signal(signal.SIGINT)
for p in procs:
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        p.kill()
