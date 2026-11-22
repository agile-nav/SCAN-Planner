#!/usr/bin/env python3
"""离线回放 scan_planner 日志，把楼梯/窄通道几何发布到 RViz。

用法：
  source /opt/ros/humble/setup.bash
  python3 tools/view_stair_log.py [日志文件] [--topic /stair_replay] [--once]

不传日志文件时自动选 ~/.ros/log 下最新的 scan_planner_node_*.log。
需要日志里有 [replan-geom]（DEBUG 级别），因此采集时建议 log_level:=debug。
"""
import argparse
import glob
import os
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

F = r'[-+]?\d+(?:\.\d+)?'
RE_GEOM = re.compile(
    rf'\[replan-geom\] rid=(\d+) ok=(\d+) robot=\(({F}),({F}),yaw=({F})\) '
    rf'goal=\(({F}),({F})\) d_goal=({F}) start=\(({F}),({F})\) '
    rf'target=\(({F}),({F})\) d_target=({F}) bearing=({F})deg '
    rf'traj_dir=\(({F}),({F})\) traj_dur=({F})')
# 当前格式：[astar] event=search_failed reason=... rid=... start=(...) end=(...)
RE_ASTAR = re.compile(
    rf'\[astar\] event=search_failed.*?rid=(-?\d+).*?'
    rf'start=\(({F}),({F}),({F})\).*?end=\(({F}),({F}),({F})\)')
# 兼容旧日志
RE_ASTAR_OLD = re.compile(
    rf'\[Astar\] SEARCH_ERR.*?start=\(({F}),({F}),({F})\).*?'
    rf'end=\(({F}),({F}),({F})\).*?rid=(\d+)')
RE_COLL = re.compile(
    rf'\[collision\] ahead=({F})s pos=\(({F}),({F}),({F})\) self_occ=(\d+)')


def pt(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def latest_log():
    candidates = glob.glob(os.path.expanduser('~/.ros/log/*/scan_planner_node_*.log'))
    candidates += glob.glob(os.path.expanduser('~/.ros/log/latest/scan_planner_node_*.log'))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def parse_log(path):
    robot, targets, astar, coll = [], [], [], []
    goal = None
    with open(path, errors='ignore') as fh:
        for line in fh:
            m = RE_GEOM.search(line)
            if m:
                v = m.groups()
                robot.append(pt(v[2], v[3], 0.0))
                targets.append(pt(v[10], v[11], 0.0))
                goal = (float(v[5]), float(v[6]))
                continue
            m = RE_ASTAR.search(line)
            if m:
                v = m.groups()  # rid, start xyz, end xyz
                astar.append(pt(v[1], v[2], v[3]))
                astar.append(pt(v[4], v[5], v[6]))
                continue
            m = RE_ASTAR_OLD.search(line)
            if m:
                v = m.groups()  # start xyz, end xyz, rid
                astar.append(pt(v[0], v[1], v[2]))
                astar.append(pt(v[3], v[4], v[5]))
                continue
            m = RE_COLL.search(line)
            if m:
                v = m.groups()
                coll.append(pt(v[1], v[2], v[3]))
    return robot, targets, astar, coll, goal


def main():
    ap = argparse.ArgumentParser(description='回放 scan_planner 日志到 RViz')
    ap.add_argument('logs', nargs='*', help='日志文件；省略则使用最新 scan_planner 日志')
    ap.add_argument('--topic', default='/stair_replay', help='输出 MarkerArray 话题')
    ap.add_argument('--frame', default='world', help='Fixed Frame')
    ap.add_argument('--rate', type=float, default=1.0, help='重发频率 Hz')
    ap.add_argument('--point-size', type=float, default=0.12, help='A*/碰撞点大小')
    ap.add_argument('--line-width', type=float, default=0.08, help='轨迹线宽')
    ap.add_argument('--once', action='store_true', help='发布一次后退出')
    ap.add_argument('--fail-only', action='store_true', help='只显示 A* 失败/碰撞点')
    args = ap.parse_args()

    paths = args.logs or [p for p in [latest_log()] if p]
    paths = [p for p in paths if os.path.isfile(p)]
    if not paths:
        print(f'找不到日志文件: {args.logs or "(未指定，且 ~/.ros/log 下也没有)"}', file=sys.stderr)
        return 2
    robot, targets, astar, coll, goal = [], [], [], [], None
    for path in paths:
        r, t, a, c, g = parse_log(path)
        robot += r; targets += t; astar += a; coll += c
        if g is not None:
            goal = g
        print(f'log={path} robot={len(r)} targets={len(t)} astar_pts={len(a)} collisions={len(c)}')
    print(f'total robot={len(robot)} targets={len(targets)} astar_pts={len(astar)} '
          f'collisions={len(coll)} goal={goal}')

    rclpy.init()
    node = Node('stair_log_viewer')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, args.topic, qos)

    def stamp():
        return node.get_clock().now().to_msg()

    def base_marker(ns, mid, mtype):
        m = Marker()
        m.header.frame_id = args.frame
        m.header.stamp = stamp()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=0).to_msg()
        return m

    def line_marker(ns, mid, points, color, width):
        m = base_marker(ns, mid, Marker.LINE_STRIP)
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.points = points
        return m

    def points_marker(ns, mid, points, color, size):
        m = base_marker(ns, mid, Marker.POINTS)
        m.scale.x = m.scale.y = size
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.points = points
        return m

    def build():
        arr = MarkerArray()
        if robot and not args.fail_only:
            arr.markers.append(line_marker('robot', 0, robot, (0.0, 1.0, 0.0, 1.0), args.line_width))
        if targets and not args.fail_only:
            arr.markers.append(line_marker('target', 1, targets, (0.2, 0.4, 1.0, 1.0), args.line_width * 0.6))
        if astar:
            arr.markers.append(points_marker('astar_fail', 2, astar, (1.0, 0.0, 0.0, 1.0), args.point_size))
        if coll:
            arr.markers.append(points_marker('collision', 3, coll, (1.0, 0.5, 0.0, 1.0), args.point_size * 1.6))
        if goal and not args.fail_only:
            m = base_marker('goal', 4, Marker.SPHERE)
            m.pose.position = pt(goal[0], goal[1], 0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            arr.markers.append(m)
        return arr

    pub.publish(build())
    if args.once:
        node.destroy_node()
        rclpy.shutdown()
        return 0

    period = 1.0 / max(args.rate, 0.1)
    node.create_timer(period, lambda: pub.publish(build()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
