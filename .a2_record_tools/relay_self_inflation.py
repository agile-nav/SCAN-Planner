#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# relay_self_inflation.py —— 回放旧 bag 时，把机身圆柱 marker 的 lifetime 洗成 0
#
# 背景：
#   旧 bag 里的 /self_inflation（机身前后两个半透明圆柱）每条都带 lifetime=0.2s。
#   录制时端到端 QoS 深度只有 1，后发的"后圆柱(id=1)"经常把先发的"前圆柱(id=0)"
#   顶掉（实测 129.6s 的包里 id=0 少了约 22%，最长的空档 3.2s）。于是回放时
#   RViz 会在这些 >0.2s 的空档里把前圆柱判过期删掉 —— 就是"前圆柱忽隐忽现"。
#
#   代码侧已经修了（db9bf2d：lifetime 0.2 -> 0.0），但已录的包改不了。
#   这个 relay 在回放链路上就地洗掉 lifetime，不用重录、不用重编译。
#
# 用法：
#   source ~/record_tools/env.sh
#   python3 relay_self_inflation.py            # 默认 /self_inflation -> /self_inflation_fixed
#   python3 relay_self_inflation.py --out-topic /self_inflation_clean
#
#   终端 1：python3 relay_self_inflation.py
#   终端 2：ros2 bag play ~/bags/<包>
#   终端 3：ros2 launch scan_planner rviz.launch.py        # 或 rviz2 -d replay.rviz
#
#   RViz 里把原来监听 /self_inflation 的那个 Marker 显示删掉（或改成 /self_inflation_fixed），
#   否则两套圆柱会叠在一起画。
#
# 两个实现要点：
#   1) lifetime 置 0（永不过期）—— 这是根因，0.2s 一到期 RViz 就删图元；
#   2) 每收到一条，就把"已知的每个 id 的最近位姿"全部重发一遍 —— 旧包里前圆柱
#      本来就有缺口，逐条转发它还是会一段段地空；补发让它停在最后一次已知位姿上。
#      另外 1 Hz 定时重发，保证 RViz 晚开也能拿到两个圆柱（latched 只留得住最后一条）。
#   3) 时间戳/坐标系一律保持原样，不做 now() 重写 —— 回放时 RViz 靠消息里的
#      stamp 去 /tf 里查变换，改成当前时间反而查不到。
# ============================================================================
import argparse
import copy
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker


def make_qos(depth, latched):
    q = QoSProfile(depth=depth)
    q.history = HistoryPolicy.KEEP_LAST
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE
    return q


class SelfInflationRelay(Node):
    def __init__(self, args):
        super().__init__("self_inflation_relay")
        self.known = {}  # id -> Marker（每个圆柱只记最近一次）
        self.n_in = 0
        self.n_out = 0

        # 输出端也挂 transient_local + 多深度：RViz 晚订阅时能一次拿到全部圆柱
        self.pub = self.create_publisher(
            Marker, args.out_topic, make_qos(max(args.depth, 2), latched=True)
        )
        self.create_subscription(
            Marker,
            args.in_topic,
            self.on_marker,
            make_qos(args.depth, latched=(args.in_qos == "latched")),
        )
        period = 1.0 / max(args.republish_hz, 0.1)
        self.create_timer(period, self.republish)

        self.get_logger().info(
            f"relay: {args.in_topic} -> {args.out_topic}  "
            f"(lifetime -> 0, in_qos={args.in_qos}, depth={args.depth}, "
            f"republish={args.republish_hz:g} Hz)"
        )

    # --- 收到一条：记住它，然后把已知的全部重发一遍 -------------------------
    def on_marker(self, msg: Marker):
        self.n_in += 1
        self.known[msg.id] = copy.deepcopy(msg)
        self.publish_all()

    def publish_all(self):
        for marker_id in sorted(self.known):
            m = copy.deepcopy(self.known[marker_id])
            # 根因就在这一行：0.2s -> 0（永不过期）
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            self.pub.publish(m)
            self.n_out += 1

    # --- 定时重发：让晚开的 RViz 也能看到两个圆柱 ---------------------------
    def republish(self):
        if self.known:
            self.publish_all()

    # --- 退出时把圆柱删掉，免得停在 RViz 里不消失 ---------------------------
    def clear(self):
        for marker_id in sorted(self.known):
            m = copy.deepcopy(self.known[marker_id])
            m.action = Marker.DELETE
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description="把 /self_inflation 的 marker lifetime 洗成 0")
    ap.add_argument("--in-topic", default="/self_inflation")
    ap.add_argument("--out-topic", default="/self_inflation_fixed")
    ap.add_argument(
        "--in-qos",
        choices=("latched", "sensor"),
        default="latched",
        help="latched=reliable+transient_local（真机/录制回放的实际情况）；"
        "若 bag 里是 best_effort 就选 sensor",
    )
    ap.add_argument("--depth", type=int, default=10, help="订阅队列深度（默认 10，别用 1）")
    ap.add_argument("--republish-hz", type=float, default=1.0, help="无新消息时的重发频率")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = SelfInflationRelay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C / pkill(SIGTERM) 都走这里，安静退出
        pass
    finally:
        # 被 pkill/SIGTERM 打断时 context 可能已经失效，这里全部吞掉异常
        try:
            node.clear()
            # 用 print 而不是 rosout：SIGTERM 之后 context 可能已失效，rosout 会报错
            print(f"[relay] 共接收 {node.n_in} 条 / 转发 {node.n_out} 条", flush=True)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
