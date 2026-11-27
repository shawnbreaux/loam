"""ROS 2 side of the carrier test: Nav2 in the ic120 namespace, a kinematic tracked IC120 that obeys
cmd_vel (with acceleration limits), the vessel's SetDumpAngle action (com3_msgs) and a pose recorder.

usage: python carrier_rig.py PARAMS OUT_DIR X Y YAW [monitor]     (inside ROS 2 Jazzy with com3_msgs built)
Writes OUT_DIR/poses.jsonl and OUT_DIR/dumps.jsonl; stops on SIGTERM.

With `monitor`, Nav2's Collision Monitor runs between the controller and the machine, fed by a simulated laser
scanner that sees an obstacle 2.5 m ahead for as long as the file OUT_DIR/obstacle exists.
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
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

PARAMS, OUT, X, Y, YAW = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:6])
MONITOR = sys.argv[6:7] == ["monitor"]
NS = "ic120"
DRIVERS = ("controller_server", "behavior_server")       # the Nav2 servers that publish velocity commands
os.makedirs(OUT, exist_ok=True)
# Start the four Nav2 servers in the machine's namespace, and the lifecycle manager that brings them up.
ns = ["--ros-args", "-r", f"__ns:=/{NS}", "--params-file", PARAMS]
servers = [("nav2_controller", "controller_server"), ("nav2_planner", "planner_server"),
           ("nav2_behaviors", "behavior_server"), ("nav2_bt_navigator", "bt_navigator")]
via: list[str] = []
if MONITOR:
    # The monitor's parameters are added (a later file wins), it becomes a fifth server, and the two servers that
    # command the tracks send their commands to it instead of to the machine.
    ns += ["--params-file", os.path.join(os.path.dirname(os.path.abspath(__file__)), "collision_monitor.yaml")]
    servers.append(("nav2_collision_monitor", "collision_monitor"))
    via = ["-r", "cmd_vel:=cmd_vel_nav"]
procs = [subprocess.Popen(["ros2", "run", pkg, exe, *ns, *(via if exe in DRIVERS else [])]) for pkg, exe in servers]
procs.append(subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", *ns, "-r",
                               "__node:=lifecycle_manager_navigation"]))

rclpy.init()
node = rclpy.create_node("ic120_sim")
# The crawler: position (m), heading (rad), speed, turn rate, and the last command with its time.
state = {"x": X, "y": Y, "th": YAW, "v": 0.0, "w": 0.0, "cmd": (0.0, 0.0), "cmd_at": 0.0}
odom_pub = node.create_publisher(Odometry, f"/{NS}/odom", 10)
tf = TransformBroadcaster(node)
static = StaticTransformBroadcaster(node)
st = TransformStamped()
# The map and odometry frames are the same here: the simulated machine has no drift.
st.header.frame_id, st.child_frame_id = "map", "odom"
st.transform.rotation.w = 1.0
static.sendTransform(st)
poses = open(f"{OUT}/poses.jsonl", "w", encoding="utf-8")  # noqa: SIM115
dumps = open(f"{OUT}/dumps.jsonl", "w", encoding="utf-8")  # noqa: SIM115


def on_cmd(m: Twist) -> None:
    """Remember the newest velocity command from Nav2 and when it came."""
    state["cmd"], state["cmd_at"] = (m.linear.x, m.angular.z), time.time()


node.create_subscription(Twist, f"/{NS}/cmd_vel", on_cmd, 10)
DT, ACC, WACC = 0.02, 0.5, 1.5                 # s, m/s^2, rad/s^2: a heavy crawler


def tick() -> None:
    """One 20 ms step of the simulated crawler: move towards the commanded speed and turn rate within the acceleration
    limits, then publish the transform and odometry and record the pose.
    """
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
    """The vessel action: record the angle asked for and where the machine stood, take a second, and succeed."""
    a = goal.request.target_angle
    dumps.write(json.dumps({"t": time.time(), "angle": a, "x": state["x"], "y": state["y"]}) + "\n")
    dumps.flush()
    time.sleep(1.0)
    goal.succeed()
    return SetDumpAngle.Result(success=True)


def scan() -> None:
    """Publish one sweep of the simulated laser scanner: nothing in sight, or, while the test's `obstacle` file
    exists, something 2.5 m straight ahead (beams within 12 degrees of the machine's heading).
    """
    m = LaserScan()
    m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), "base_link"
    m.angle_min, m.angle_max, m.angle_increment = -math.pi, math.pi, 2 * math.pi / 360
    m.range_min, m.range_max = 0.1, 50.0
    seen = os.path.exists(f"{OUT}/obstacle")
    m.ranges = [2.5 if seen and abs(k - 180) <= 12 else math.inf for k in range(360)]
    scan_pub.publish(m)


ActionServer(node, SetDumpAngle, f"/{NS}/set_dump_angle", dump)
if MONITOR:
    scan_pub = node.create_publisher(LaserScan, f"/{NS}/scan", 10)
    node.create_timer(0.1, scan)
node.create_timer(DT, tick)
ex = MultiThreadedExecutor()
ex.add_node(node)
running = True
# SIGTERM ends the loop below, so the files are closed and Nav2 is shut down in order.
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
