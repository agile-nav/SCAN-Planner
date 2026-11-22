#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# analyze_run.py —— 解析 record_run.sh 录下来的 run.log + bag，产出 CSV / 图 / 摘要
#
# 用法（先 source ROS 与工作空间；解析 Bspline 自定义类型需要）：
#   source /opt/ros/humble/setup.bash
#   source ~/SCAN-Planner/install/setup.bash
#   python3 analyze_run.py ~/clock_sync_runs/<run_dir>
#
# 产物（<run_dir>/analysis/）：
#   traj_clock.csv        [traj-clock] 全部字段
#   ctl_clock.csv         [ctl-clock] 全部字段
#   events.csv            事件（stuck / drop / 状态迁移 / 到达 …）
#   cmd_vs_odom.csv       bag 里的指令速度 vs 实测速度（长表）
#   flags.csv            冻结 / 使能（长表）
#   trajectories.csv      bag 里的每条 B 样条（时长、点数）
#   status.csv            /local_planner/status 的状态序列
#   summary.txt           指标 + 判据
#   plots/*.png           曲线图
# ============================================================================
import argparse
import csv
import json
import math
import os
import re
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
LOG_RE = re.compile(r"^\[(INFO|WARN|ERROR|DEBUG|FATAL)\]\s+\[([0-9.]+)\]\s+\[([^\]]+)\]:\s*(.*)$")
# ros2 launch 会给子进程输出加 "[scan_planner_node-1] " 前缀，解析前先剥掉
LAUNCH_PREFIX = re.compile(r"^\[[A-Za-z0-9_./-]+-\d+\]\s+")
TRAJ_RE = re.compile(
    r"\[traj-clock\]\s+t_wall=([-\d.]+)\s+t_proj=([-\d.]+)\s+t_cur=([-\d.]+)\s+"
    r"lead=([-\d.]+)\s+max_lead=([-\d.]+)\s+dur=([-\d.]+)\s+traj_id=(\d+)")
CTL_RE = re.compile(
    r"\[ctl-clock\]\s+exec_time=([-\d.]+)\s+t_proj=([-\d.]+)\s+lead=([-\d.]+)\s+"
    r"dur=([-\d.]+)\s+pos_err=([-\d.]+)\s+traj_id=(\d+)")
RECV_RE = re.compile(r"Received trajectory\s+(\d+),\s+duration\s+([\d.]+)s")
# 兼容旧 ROS1 风格裸输出
FSM_OLD_RE = re.compile(r"^\[FSM\]:\s+state:\s+(\w+)$")
TRANS_RE = re.compile(r"^\[([^\]]+)\]:\s+from\s+(\w+)\s+to\s+(\w+)$")
# 当前 [fsm] event=... 结构化日志
FSM_STATE_RE = re.compile(r"^\[fsm\] event=state from=(\w+) to=(\w+) trigger=(\S+)$")
FSM_EVENT_RE = re.compile(r"^\[fsm\] event=([a-z_]+)(?:\s+(.*))?$")
REPLAN_FAIL_RE = re.compile(
    r"^\[fsm\] event=replan_failed rid=(\d+) traj_id=(-?\d+) reason=(\S+) "
    r"replan_fail=(\d+)/(\d+)$")


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round((len(s) - 1) * q))))
    return s[i]


def read_user_events(run_dir):
    """读取 record_run --event 写入的 events.tsv。"""
    events = []
    path = os.path.join(run_dir, "events.tsv")
    if not os.path.isfile(path):
        return events
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                parts = line.rstrip("\n").split("\t", 1)
                if len(parts) != 2:
                    continue
                try:
                    t = float(parts[0])
                except ValueError:
                    continue
                events.append({"t": f"{t:.6f}", "approx": 0,
                               "kind": "user_event", "detail": parts[1]})
    except OSError:
        pass
    return events


def read_params(path):
    out = {}
    if not os.path.isfile(path):
        return out
    try:
        for line in open(path, encoding="utf-8", errors="replace"):
            m = re.match(r"\s*([\w.]+):\s*([-\d.]+)\s*$", line)
            if m:
                out[m.group(1)] = float(m.group(2))
    except OSError:
        pass
    return out


def parse_log(path):
    traj, ctl, events = [], [], []
    last_t = None
    span = [None, None]

    def ev(t, approx, kind, detail):
        events.append({"t": "" if t is None else f"{t:.6f}",
                       "approx": int(approx), "kind": kind, "detail": detail})

    if not os.path.isfile(path):
        return traj, ctl, events, span
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = ANSI.sub("", raw).rstrip("\n")
            s = line.strip()
            if not s:
                continue
            s = LAUNCH_PREFIX.sub("", s)
            m = LOG_RE.match(s)
            if m:
                t = float(m.group(2))
                msg = m.group(4)
                if span[0] is None:
                    span[0] = t
                span[1] = t
                last_t = t
                mm = TRAJ_RE.search(msg)
                if mm:
                    traj.append(dict(zip(
                        ["t", "t_wall", "t_proj", "t_cur", "lead", "max_lead", "dur", "traj_id"],
                        [t] + [float(x) for x in mm.groups()[:6]] + [int(mm.group(7))])))
                    continue
                mm = CTL_RE.search(msg)
                if mm:
                    ctl.append(dict(zip(
                        ["t", "exec_time", "t_proj", "lead", "dur", "pos_err", "traj_id"],
                        [t] + [float(x) for x in mm.groups()[:5]] + [int(mm.group(6))])))
                    continue
                mm = RECV_RE.search(msg)
                if mm:
                    ev(t, 0, "received_traj", f"traj_id={mm.group(1)} dur={mm.group(2)}")
                if "Robot stuck" in msg:
                    ev(t, 0, "robot_stuck", msg)
                if "trajectory older than" in msg:
                    ev(t, 0, "traj_timeout_drop", msg)
                if "Obstacle discovered" in msg:
                    ev(t, 0, "obstacle_emergency", msg)
                if msg.startswith("motion_enabled="):
                    ev(t, 0, "motion_enabled", msg)
                if "External STOP" in msg or "External RESUME" in msg:
                    ev(t, 0, "mission_control", msg)
                if "Reference TF" in msg:
                    ev(t, 0, "reference_tf", msg)
                mm = FSM_STATE_RE.search(msg)
                if mm:
                    ev(t, 0, "fsm_transition", f"{mm.group(1)}->{mm.group(2)} ({mm.group(3)})")
                mm = REPLAN_FAIL_RE.search(msg)
                if mm:
                    ev(t, 0, "replan_failed",
                       f"rid={mm.group(1)} traj_id={mm.group(2)} reason={mm.group(3)} "
                       f"fail={mm.group(4)}/{mm.group(5)}")
                else:
                    mm = FSM_EVENT_RE.search(msg)
                    if mm and mm.group(1) not in ("state",):
                        ev(t, 0, "fsm_" + mm.group(1), (mm.group(2) or "").strip())
            else:
                # 裸输出（cout）：没有时间戳，用“上一条结构化日志的时间”近似
                mm = FSM_OLD_RE.match(s)
                if mm:
                    ev(last_t, 1, "fsm_state", mm.group(1))
                    continue
                mm = TRANS_RE.match(s)
                if mm:
                    ev(last_t, 1, "fsm_transition", f"{mm.group(2)}->{mm.group(3)} ({mm.group(1)})")
                    continue
                if s.startswith("[rebo replan]"):
                    ev(last_t, 1, "rebo_replan", s)
                elif s.startswith("final_plan_success="):
                    ev(last_t, 1, "final_plan_success", s)
                elif s.startswith("first_optimize_step_success="):
                    ev(last_t, 1, "optimize_step", s)
                elif s.startswith("total time:"):
                    ev(last_t, 1, "total_time", s)
                elif s.startswith("Close to goal"):
                    ev(last_t, 1, "close_to_goal", s)
                elif s.startswith("Triggered!"):
                    ev(last_t, 1, "triggered", s)
                elif "Need to reallocate time" in s:
                    ev(last_t, 1, "reallocate_time", s)
                elif "This refined trajectory is unsafe" in s:
                    ev(last_t, 1, "refined_unsafe", s)
    return traj, ctl, events, span


def bag_storage_id(bag_dir):
    meta = os.path.join(bag_dir, "metadata.yaml")
    if os.path.isfile(meta):
        try:
            txt = open(meta, encoding="utf-8", errors="replace").read()
            m = re.search(r"storage_identifier:\s*(\w+)", txt)
            if m:
                return m.group(1)
        except OSError:
            pass
    return "sqlite3"


def read_bag(bag_dir, odom_topics):
    """返回 dict: cmd/odom/traj/frozen/enabled/status，每个是 [(t, value...)]。
    读不到就返回空 dict（不抛异常）。"""
    out = {"cmd": [], "odom": [], "traj": [], "frozen": [], "enabled": [], "status": [],
           "diagnostics": []}
    if not os.path.isdir(bag_dir):
        return out
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as e:  # noqa
        print(f"[bag] 跳过：无法 import rosbag2（{e}）。先 source ROS 与工作空间。")
        return out
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_dir, storage_id=bag_storage_id(bag_dir)),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"))
        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        chosen_odom = next((x for x in odom_topics if x in type_map), None)
        while reader.has_next():
            topic, data, t = reader.read_next()
            t = t / 1e9
            try:
                msg = deserialize_message(data, get_message(type_map[topic]))
            except Exception:
                continue
            if topic == "/cmd_vel":
                out["cmd"].append((t, math.hypot(msg.linear.x, msg.linear.y)))
            elif chosen_odom and topic == chosen_odom:
                tw = msg.twist.twist.linear
                out["odom"].append((t, math.hypot(tw.x, tw.y)))
            elif topic == "/planning/bspline":
                try:
                    n = len(msg.pos_pts)
                    p = int(msg.order)
                    knots = list(msg.knots)
                    dur = knots[n] - knots[p] if len(knots) > n else float("nan")
                except Exception:
                    n, p, dur = 0, 0, float("nan")
                out["traj"].append((t, int(msg.traj_id), dur, n))
            elif topic == "/planning/go2_execution_frozen":
                out["frozen"].append((t, bool(msg.data)))
            elif topic == "/motion_enabled":
                out["enabled"].append((t, bool(msg.data)))
            elif topic == "/local_planner/status":
                state = ""
                try:
                    state = json.loads(msg.data).get("state", "")
                except Exception:
                    state = str(msg.data)
                out["status"].append((t, state))
            elif topic == "/local_planner/diagnostics":
                try:
                    out["diagnostics"].append((t, json.loads(msg.data)))
                except Exception:
                    pass
    except Exception as e:  # noqa
        print(f"[bag] 读取失败：{e}")
    for k in out:
        out[k].sort(key=lambda x: x[0])
    return out


def write_csv(path, rows, fields):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_jsonl(path, rows):
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


def nearest(series, t, max_dt=0.5):
    """series: [(t, value...)] 已排序；返回 t 附近最近的一条。"""
    if not series:
        return None
    best = None
    best_d = 1e9
    for item in series:
        d = abs(item[0] - t)
        if d < best_d:
            best_d = d
            best = item
        if item[0] > t and d > best_d:
            break
    return best if best_d <= max_dt else None


def stuck_delays(bag, stuck_times, cmd_eps=0.15, spd_eps=0.03):
    """用 bag 的 cmd/odom 估计“想走但不动”区间的起点，给出检测延迟。"""
    cmd = bag.get("cmd", [])
    odom = bag.get("odom", [])
    if not cmd or not odom or not stuck_times:
        return []
    merged = []
    for t, v in cmd:
        o = nearest(odom, t, 0.15)
        if o is not None:
            merged.append((t, v, o[1]))
    merged.sort()
    delays = []
    for ts in stuck_times:
        i = len(merged) - 1
        while i >= 0 and merged[i][0] > ts:
            i -= 1
        if i < 0:
            continue
        j = i
        while j >= 0 and merged[j][1] > cmd_eps and merged[j][2] < spd_eps:
            j -= 1
        start = merged[j + 1][0] if j + 1 <= i else merged[i][0]
        if merged[i][0] - start >= 0.05:
            delays.append(ts - start)
    return delays


def post_goal_cmd(bag, transitions, window=5.0):
    """状态进入 goal_reached/idle 后，cmd 归零所需时间。"""
    cmd = bag.get("cmd", [])
    if not cmd or not transitions:
        return []
    res = []
    for t0 in transitions:
        after = [(t, v) for t, v in cmd if t >= t0 and t - t0 <= window]
        if not after:
            continue
        zero_since = None
        done = None
        for t, v in after:
            if v <= 0.05:
                if zero_since is None:
                    zero_since = t
                if t - zero_since >= 0.2:
                    done = t
                    break
            else:
                zero_since = None
        res.append((done - t0) if done is not None else window)
    return res


def main():
    ap = argparse.ArgumentParser(description="分析 clock_sync 录制结果")
    ap.add_argument("run_dir", nargs="?",
                    help="run 目录；省略时使用 $CLOCK_SYNC_ROOT/current")
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--no-bag", action="store_true")
    ap.add_argument("--json", action="store_true", help="额外输出 analysis/events.jsonl")
    ap.add_argument("--no-plot", action="store_true", help="只生成 CSV/摘要，不画图")
    args = ap.parse_args()

    if not args.run_dir:
        root = os.environ.get("CLOCK_SYNC_ROOT", os.path.expanduser("~/clock_sync_runs"))
        args.run_dir = os.path.join(root, "current")
    if not os.path.isdir(args.run_dir):
        print(f"run 目录不存在: {args.run_dir}", file=sys.stderr)
        return 2

    run_dir = os.path.abspath(args.run_dir)
    ana = os.path.join(run_dir, "analysis")
    os.makedirs(ana, exist_ok=True)
    log_path = os.path.join(run_dir, "run.log")
    bag_dir = os.path.join(run_dir, "bag")

    traj, ctl, events, span = parse_log(log_path)
    events += read_user_events(run_dir)
    events.sort(key=lambda e: float(e["t"]) if e["t"] else -1e18)
    write_csv(os.path.join(ana, "traj_clock.csv"), traj,
              ["t", "t_wall", "t_proj", "t_cur", "lead", "max_lead", "dur", "traj_id"])
    write_csv(os.path.join(ana, "ctl_clock.csv"), ctl,
              ["t", "exec_time", "t_proj", "lead", "dur", "pos_err", "traj_id"])
    write_csv(os.path.join(ana, "events.csv"), events, ["t", "approx", "kind", "detail"])
    if args.json:
        write_jsonl(os.path.join(ana, "events.jsonl"), events)

    bag = {"cmd": [], "odom": [], "traj": [], "frozen": [], "enabled": [], "status": [],
           "diagnostics": []}
    if not args.no_bag:
        bag = read_bag(bag_dir, [args.odom_topic, "/lio/robo/odom", "/body_pose", "/loc/robo/odom"])
        write_csv(os.path.join(ana, "cmd_vs_odom.csv"),
                  [{"t": t, "series": "cmd", "value": v} for t, v in bag["cmd"]] +
                  [{"t": t, "series": "odom", "value": v} for t, v in bag["odom"]],
                  ["t", "series", "value"])
        write_csv(os.path.join(ana, "flags.csv"),
                  [{"t": t, "flag": "frozen", "value": int(v)} for t, v in bag["frozen"]] +
                  [{"t": t, "flag": "enabled", "value": int(v)} for t, v in bag["enabled"]],
                  ["t", "flag", "value"])
        write_csv(os.path.join(ana, "trajectories.csv"),
                  [{"t": t, "traj_id": i, "duration": d, "n_points": n} for t, i, d, n in bag["traj"]],
                  ["t", "traj_id", "duration", "n_points"])
        write_csv(os.path.join(ana, "status.csv"),
                  [{"t": t, "state": s} for t, s in bag["status"]],
                  ["t", "state"])
        diag_rows = []
        for t, d in bag["diagnostics"]:
            row = {"t": t}
            for key in ("state", "rid", "traj_id", "replans", "replan_ok", "replan_fail",
                        "degrade", "backup", "backup_short", "last_reason",
                        "plan_ms_last", "plan_ms_avg", "plan_ms_max"):
                row[key] = d.get(key, "")
            diag_rows.append(row)
        write_csv(os.path.join(ana, "diagnostics.csv"), diag_rows,
                  ["t", "state", "rid", "traj_id", "replans", "replan_ok",
                   "replan_fail", "degrade", "backup", "backup_short",
                   "last_reason", "plan_ms_last", "plan_ms_avg", "plan_ms_max"])

    # ---------- 统计 ----------
    lines = []
    def p(*a):
        s = " ".join(str(x) for x in a)
        lines.append(s)
        print(s)

    span_s = (span[1] - span[0]) if span[0] is not None and span[1] is not None else 0.0
    p(f"run_dir: {run_dir}")
    p(f"log span: {span_s:.1f} s")

    def stats(name, rows, key):
        vals = [r[key] for r in rows if not math.isnan(r[key])]
        if not vals:
            p(f"{name}: 无数据")
            return
        p(f"{name}: n={len(vals)} p50={pct(vals,.5):.2f} p95={pct(vals,.95):.2f} max={max(vals):.2f}")

    stats("[traj-clock] lead", traj, "lead")
    stats("[ctl-clock]  lead", ctl, "lead")

    if traj:
        rates = len(traj) / span_s if span_s > 0 else 0
        p(f"[traj-clock] 频率: {rates:.2f} Hz（目标 ~1 Hz）")
        ml = sorted({round(r['max_lead'], 2) for r in traj})
        p(f"[traj-clock] max_lead 取值: {ml}")
    else:
        p("[traj-clock] 没有数据 —— 检查是否有轨迹 / have_odom / 是否用了 G3 之后的分支")

    ev = [e for e in events if e["kind"] == "robot_stuck"]
    stuck_ts = [float(e["t"]) for e in ev if e["t"] != ""]
    p(f"Robot stuck 次数: {len(ev)}")
    delays = stuck_delays(bag, stuck_ts)
    if delays:
        p(f"卡住检测延迟: n={len(delays)} p50={pct(delays,.5):.2f}s max={max(delays):.2f}s（期望 <=1.3s）")
    elif ev:
        p("卡住检测延迟: 无法从 bag 估计（缺 /cmd_vel 或 odom）")

    drops = [e for e in events if e["kind"] == "traj_timeout_drop"]
    p(f"trajectory older than …: {len(drops)} 次")
    if drops and bag["frozen"]:
        during = 0
        for e in drops:
            if e["t"] == "":
                continue
            f = nearest(bag["frozen"], float(e["t"]), 0.5)
            if f and f[1]:
                during += 1
        p(f"  其中发生在冻结期间: {during} 次（期望 0）")

    # 失败原因与链路
    fails = [e for e in events if e["kind"] == "replan_failed"]
    p(f"replan_failed 次数: {len(fails)}")
    if fails:
        from collections import Counter
        reasons = []
        for e in fails:
            mm = re.search(r"reason=(\S+)", e["detail"])
            if mm:
                reasons.append(mm.group(1))
        p("失败原因分布:")
        for reason, count in Counter(reasons).most_common():
            p(f"  {reason:20s} {count}")
        last = fails[-1]
        rid_m = re.search(r"rid=(\d+)", last["detail"])
        if rid_m:
            rid = rid_m.group(1)
            p(f"最后一次失败链 (rid={rid}):")
            related = [e for e in events if f"rid={rid}" in e["detail"]]
            for e in related[-10:]:
                p(f"  [{e['t']}] {e['kind']}: {e['detail']}")
    deg = [e for e in events if e["kind"] == "fsm_goal_degraded"]
    p(f"目标降级次数: {len(deg)}")
    done = [e for e in events if e["kind"] == "fsm_mission_done"]
    p(f"任务完成次数: {len(done)}")
    user_events = [e for e in events if e["kind"] == "user_event"]
    p(f"人工事件次数: {len(user_events)}")

    if bag["diagnostics"]:
        d = bag["diagnostics"][-1][1]
        p(f"diagnostics final: state={d.get('state','')} replans={d.get('replans','')} "
          f"ok={d.get('replan_ok','')} fail={d.get('replan_fail','')} "
          f"degrade={d.get('degrade','')} backup={d.get('backup','')} "
          f"last_reason={d.get('last_reason','')}")
        failures = d.get("failures", {}) or {}
        if failures:
            p("diagnostics failure counts:")
            for reason, count in sorted(failures.items(), key=lambda x: -x[1]):
                p(f"  {reason:24s} {count}")
        vals = []
        for _, item in bag["diagnostics"]:
            try:
                vals.append(float(item.get("plan_ms_last", 0.0)))
            except (TypeError, ValueError):
                pass
        if vals:
            p(f"plan_ms_last: n={len(vals)} p50={pct(vals,.5):.1f} "
              f"p95={pct(vals,.95):.1f} max={max(vals):.1f}")

    # 3.000 s 轨迹
    trajs3 = [x for x in bag["traj"] if abs(x[2] - 3.0) < 0.01]
    p(f"3.000 s 轨迹: {len(trajs3)} 条 / 总轨迹 {len(bag['traj'])} 条")
    trans = []
    prev = None
    for t, s in bag["status"]:
        if prev is not None and s != prev and s in ("goal_reached", "idle"):
            trans.append(t)
        prev = s
    if trans:
        p(f"到达/空闲状态迁移: {len(trans)} 次 -> {['%.1f' % x for x in trans[:8]]}"
          + (" ..." if len(trans) > 8 else ""))
        near3 = 0
        for t0 in trans:
            if any(abs(tt - t0) <= 1.0 for tt, *_ in trajs3):
                near3 += 1
        p(f"  其中收尾后 1s 内出现 3.000 s 轨迹: {near3} 次（P1 预期会出现）")
        pg = post_goal_cmd(bag, trans)
        if pg:
            p(f"收尾后 cmd 归零时间: n={len(pg)} p50={pct(pg,.5):.2f}s max={max(pg):.2f}s（期望 <=0.3s）")
    elif bag["status"]:
        p("未看到 goal_reached/idle 迁移（可能没跑到目标，或状态话题没录到）")

    # ---------- 摘要文件 ----------
    with open(os.path.join(ana, "summary.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # ---------- 画图（可选） ----------
    if args.no_plot:
        print("[plot] 已跳过（--no-plot）")
        return 0
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa
        print(f"[plot] 跳过：matplotlib 不可用（{e}）；先 pip install matplotlib")
        return 0

    plots = os.path.join(ana, "plots")
    os.makedirs(plots, exist_ok=True)

    def xs(rows, key):
        return [r["t"] for r in rows], [r[key] for r in rows]

    if traj:
        plt.figure(figsize=(10, 4))
        t, y = xs(traj, "t_cur"); plt.plot(t, y, label="t_cur")
        t, y = xs(traj, "t_proj"); plt.plot(t, y, label="t_proj")
        t, y = xs(traj, "lead"); plt.plot(t, y, label="lead")
        ml = pct([r["max_lead"] for r in traj], .5)
        if not math.isnan(ml):
            plt.axhline(ml, ls="--", lw=0.8, color="gray", label=f"max_lead={ml:.2f}")
        plt.xlabel("time (s)"); plt.ylabel("s"); plt.legend(); plt.grid(alpha=.3)
        plt.title("[traj-clock]"); plt.tight_layout()
        plt.savefig(os.path.join(plots, "traj_clock.png"), dpi=150); plt.close()

    if ctl:
        plt.figure(figsize=(10, 4))
        t, y = xs(ctl, "exec_time"); plt.plot(t, y, label="exec_time")
        t, y = xs(ctl, "t_proj"); plt.plot(t, y, label="t_proj")
        t, y = xs(ctl, "lead"); plt.plot(t, y, label="lead")
        params = read_params(os.path.join(run_dir, "params_controller.yaml"))
        tl = params.get("time_lead", 0.2)
        plt.axhline(tl, ls="--", lw=0.8, color="gray", label=f"time_lead={tl:.2f}")
        plt.xlabel("time (s)"); plt.ylabel("s"); plt.legend(); plt.grid(alpha=.3)
        plt.title("[ctl-clock]"); plt.tight_layout()
        plt.savefig(os.path.join(plots, "ctl_clock.png"), dpi=150); plt.close()

    if bag["cmd"] or bag["odom"]:
        plt.figure(figsize=(10, 4))
        if bag["cmd"]:
            t, y = zip(*bag["cmd"]); plt.plot(t, y, label="cmd |v|")
        if bag["odom"]:
            t, y = zip(*bag["odom"]); plt.plot(t, y, label="odom |v|")
        # 冻结区间上色
        for i in range(1, len(bag["frozen"])):
            if bag["frozen"][i][1] and not bag["frozen"][i - 1][1]:
                start = bag["frozen"][i][0]
                end = start
                for j in range(i, len(bag["frozen"])):
                    if not bag["frozen"][j][1]:
                        end = bag["frozen"][j][0]; break
                    end = bag["frozen"][j][0]
                plt.axvspan(start, end, color="orange", alpha=.15)
        plt.xlabel("time (s)"); plt.ylabel("m/s"); plt.legend(); plt.grid(alpha=.3)
        plt.title("cmd vs odom（橙=冻结）"); plt.tight_layout()
        plt.savefig(os.path.join(plots, "cmd_vs_odom.png"), dpi=150); plt.close()

    if bag["traj"]:
        durs = [x[2] for x in bag["traj"] if not math.isnan(x[2])]
        plt.figure(figsize=(8, 4))
        plt.hist(durs, bins=30, color="#4c78a8", alpha=.85)
        plt.axvline(3.0, color="orange", ls="--", label="3.000 s（停车轨迹）")
        plt.xlabel("trajectory duration (s)"); plt.ylabel("count")
        plt.legend(); plt.grid(alpha=.3)
        plt.title("B-spline durations"); plt.tight_layout()
        plt.savefig(os.path.join(plots, "traj_durations.png"), dpi=150); plt.close()

    print(f"[plot] 图已保存到 {plots}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
