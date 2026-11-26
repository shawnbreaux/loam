"""ROS 2 side of the carrier test: Nav2 in the ic120 namespace, a kinematic tracked IC120 that obeys
cmd_vel (with acceleration limits), the vessel's SetDumpAngle action (com3_msgs) and a pose recorder.

usage: python carrier_rig.py PARAMS OUT_DIR X Y YAW      (inside ROS 2 Jazzy with com3_msgs built)
Writes OUT_DIR/poses.jsonl and OUT_DIR/dumps.jsonl; stops on SIGTERM.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from com3_msgs.action import SetDumpAngle
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

PARAMS, OUT, X, Y, YAW = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:6])
NS = "ic120"
os.makedirs(OUT, exist_ok=True)
ns = ["--ros-args", "-r", f"__ns:=/{NS}", "--params-file", PARAMS]
procs = [subprocess.Popen(["ros2", "run", pkg, exe, *ns]) for pkg, exe in (
    ("nav2_controller", "controller_server"), ("nav2_planner", "planner_server"),
    ("nav2_behaviors", "behavior_server"), ("nav2_bt_navigator", "bt_navigator"))]
procs.append(subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-r",
                               f"__ns:=/{NS}", "-r", "__node:=lifecycle_manager_navigation",
                               "--params-file", PARAMS]))

rclpy.init()
node = rclpy.create_node("ic120_sim")
state = {"x": X, "y": Y, "th": YAW, "v": 0.0, "w": 0.0, "cmd": (0.0, 0.0), "cmd_at": 0.0}
odom_pub = node.create_publisher(Odometry, f"/{NS}/odom", 10)
tf = TransformBroadcaster(node)
static = StaticTransformBroadcaster(node)
st = TransformStamped()
st.header.frame_id, st.child_frame_id = "map", "odom"
st.transform.rotation.w = 1.0
static.sendTransform(st)
poses = open(f"{OUT}/poses.jsonl", "w", encoding="utf-8")  # noqa: SIM115
dumps = open(f"{OUT}/dumps.jsonl", "w", encoding="utf-8")  # noqa: SIM115


def on_cmd(m: Twist) -> None:
    state["cmd"], state["cmd_at"] = (m.linear.x, m.angular.z), time.time()


node.create_subscription(Twist, f"/{NS}/cmd_vel", on_cmd, 10)
DT, ACC, WACC = 0.02, 0.5, 1.5                 # s, m/s^2, rad/s^2: a heavy crawler


def tick() -> None:
    cv, cw = state["cmd"] if time.time() - state["cmd_at"] < 0.5 else (0.0, 0.0)   # no command: stop
    state["v"] += max(-ACC * DT, min(ACC * DT, cv - state["v"]))
    state["w"] += max(-WACC * DT, min(WACC * DT, cw - state["w"]))
    state["th"] += state["w"] * DT
    state["x"] += state["v"] * math.cos(state["th"]) * DT
    state["y"] += state["v"] * math.sin(state["th"]) * DT
    now = node.get_clock().now().to_msg()
    t = TransformStamped()
    t.header.stamp, t.header.frame_id, t.child_frame_id = now, "odom", "base_link"
    t.transform.translation.x, t.transform.translation.y = state["x"], state["y"]
    t.transform.rotation.z, t.transform.rotation.w = math.sin(state["th"] / 2), math.cos(state["th"] / 2)
    tf.sendTransform(t)
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = now, "odom", "base_link"
    o.pose.pose.position.x, o.pose.pose.position.y = state["x"], state["y"]
    o.pose.pose.orientation = t.transform.rotation
    o.twist.twist.linear.x, o.twist.twist.angular.z = state["v"], state["w"]
    if not os.path.exists(f"{OUT}/odom_off"):  # the test cuts the link by creating this file
        odom_pub.publish(o)
    poses.write(json.dumps({"t": time.time(), "x": state["x"], "y": state["y"], "v": state["v"]}) + "\n")


def dump(goal):
    a = goal.request.target_angle
    dumps.write(json.dumps({"t": time.time(), "angle": a, "x": state["x"], "y": state["y"]}) + "\n")
    dumps.flush()
    time.sleep(1.0)
    goal.succeed()
    return SetDumpAngle.Result(success=True)


ActionServer(node, SetDumpAngle, f"/{NS}/set_dump_angle", dump)
node.create_timer(DT, tick)
ex = MultiThreadedExecutor()
ex.add_node(node)
running = True
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("running", False))
print("SIM UP", flush=True)
while running:
    ex.spin_once(timeout_sec=0.05)
poses.close()
dumps.close()
for p in procs:
    p.send_signal(signal.SIGINT)
for p in procs:
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        p.kill()
