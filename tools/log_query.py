#!/usr/bin/env python3
# ============================================================================
# 文件：tools/log_query.py —— 日志查询 / 统计 / 导出
#
# 用法（命令在前）：
#   python3 tools/log_query.py summary run.log
#   python3 tools/log_query.py rid 42 run.log
#   python3 tools/log_query.py last-failure run.log
#   python3 tools/log_query.py errors run.log
#   python3 tools/log_query.py slow run.log
#   python3 tools/log_query.py timeline run.log
#   python3 tools/log_query.py export run.log --out run.jsonl
#
# 也支持 "run.log summary" 这种路径在前的写法。
# ============================================================================
import argparse
import glob
import json
import os
import re
import sys
from collections import Counter

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
LAUNCH_PREFIX = re.compile(r"^\[[A-Za-z0-9_./-]+-\d+\]\s+")
LOG_RE = re.compile(r"^\[(INFO|WARN|ERROR|DEBUG|FATAL)\]\s+\[([0-9.]+)\]\s+\[([^\]]+)\]:\s*(.*)$")
TAG_EVENT_RE = re.compile(r"^\[([a-z_]+)\]\s*(.*)$")
KV_RE = re.compile(r"(\w+)=(\"[^\"]*\"|[^\s]+)")
RID_RE = re.compile(r"\brid=(-?\d+)\b")

COMMANDS = {"summary", "rid", "last-failure", "errors", "slow", "timeline", "export"}


def parse_file(path, out):
    if not os.path.isfile(path):
        print(f"[warn] 文件不存在: {path}", file=sys.stderr)
        return
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            s = ANSI.sub("", raw).strip()
            if not s:
                continue
            s = LAUNCH_PREFIX.sub("", s)
            m = LOG_RE.match(s)
            if m:
                level, t, node, msg = m.group(1), float(m.group(2)), m.group(3), m.group(4)
            else:
                level, t, node, msg = "", None, "", s
            tag, event = "", ""
            tm = TAG_EVENT_RE.match(msg)
            body = msg
            if tm:
                tag, body = tm.group(1), tm.group(2)
            fields = {k: v.strip('"') for k, v in KV_RE.findall(msg)}
            event = fields.get("event", "")
            out.append({
                "t": t,
                "level": level,
                "node": node,
                "tag": tag,
                "event": event,
                "fields": fields,
                "msg": msg,
                "file": path,
            })


def default_paths():
    root = os.environ.get("CLOCK_SYNC_ROOT", os.path.expanduser("~/clock_sync_runs"))
    current = os.path.join(root, "current", "run.log")
    if os.path.isfile(current):
        return [current]
    candidates = glob.glob(os.path.expanduser("~/.ros/log/latest/scan_planner_node_*.log"))
    candidates += glob.glob(os.path.expanduser("~/.ros/log/*/scan_planner_node_*.log"))
    if candidates:
        return [max(candidates, key=os.path.getmtime)]
    return []


def load(paths):
    if not paths:
        paths = default_paths()
    records = []
    for pattern in paths:
        matched = glob.glob(pattern) or [pattern]
        for path in sorted(matched):
            parse_file(path, records)
    return records


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round((len(s) - 1) * q))))
    return s[i]


def fmt_t(t):
    return "        " if t is None else f"{t:9.3f}"


def print_record(r):
    print(f"{fmt_t(r['t'])} {r['level']:5s} {r['tag']:7s} {r['event']:20s} {r['msg']}")


def cmd_summary(records, args):
    ts = [r["t"] for r in records if r["t"] is not None]
    span = (max(ts) - min(ts)) if ts else 0.0
    print(f"records: {len(records)}  span: {span:.1f}s")
    print("levels:")
    for level, count in Counter(r["level"] or "(bare)" for r in records).most_common():
        print(f"  {level:8s} {count}")
    events = Counter(r["event"] for r in records if r["event"])
    print("top events:")
    for event, count in events.most_common(15):
        print(f"  {event:24s} {count}")
    fails = [r for r in records if r["event"] == "replan_failed"]
    if fails:
        reasons = Counter(r["fields"].get("reason", "?") for r in fails)
        print(f"replan_failed: {len(fails)}")
        for reason, count in reasons.most_common():
            print(f"  {reason:24s} {count}")
        by_rid = Counter(r["fields"].get("rid", "?") for r in fails)
        print("failures by rid:")
        for rid, count in by_rid.most_common(10):
            print(f"  rid={rid:8s} {count}")
        times = sorted(r["t"] for r in fails if r["t"] is not None)
        gaps = [b - a for a, b in zip(times, times[1:]) if b >= a]
        if gaps:
            print(f"retry gap: n={len(gaps)} p50={pct(gaps,.5):.2f}s p95={pct(gaps,.95):.2f}s")
    plan_ms = []
    for r in records:
        for key in ("plan_ms_last", "plan_ms_avg"):
            if key in r["fields"]:
                try:
                    plan_ms.append(float(r["fields"][key]))
                except ValueError:
                    pass
                break
    if plan_ms:
        print(f"plan_ms: n={len(plan_ms)} p50={pct(plan_ms,.5):.1f} "
              f"p95={pct(plan_ms,.95):.1f} max={max(plan_ms):.1f}")
    ids = {r["fields"].get("rid") for r in records if r["fields"].get("rid")}
    print(f"distinct rid: {len(ids)}")
    for key in ("goal_degraded", "backup_start", "backup_done", "mission_done", "mission_abandoned"):
        n = sum(1 for r in records if r["event"] == key)
        if n:
            print(f"{key}: {n}")


def cmd_rid(records, args):
    rid = str(args.rid)
    selected = [r for r in records
                if r["fields"].get("rid") == rid or RID_RE.search(r["msg"]) and RID_RE.search(r["msg"]).group(1) == rid]
    if not selected:
        print(f"没有找到 rid={rid}")
        return
    for r in selected:
        print_record(r)


def cmd_last_failure(records, args):
    fails = [r for r in records if r["event"] == "replan_failed"]
    if not fails:
        print("没有 replan_failed 记录")
        return
    last = fails[-1]
    rid = last["fields"].get("rid")
    print(f"last failure: {last['msg']}")
    if not rid:
        return
    print(f"chain (rid={rid}):")
    for r in records:
        if r["fields"].get("rid") == rid:
            print_record(r)


def cmd_errors(records, args):
    selected = [r for r in records if r["level"] in ("WARN", "ERROR", "FATAL")]
    for r in selected[-args.limit:]:
        print_record(r)


def cmd_slow(records, args):
    rows = []
    for r in records:
        fields = r["fields"]
        value = None
        for key in ("elapsed", "total_ms", "time_ms", "plan_ms_last", "plan_ms_avg", "total"):
            if key in fields:
                try:
                    value = float(fields[key].rstrip("s"))
                except ValueError:
                    value = None
                break
        if value is not None:
            rows.append((value, r))
    if not rows:
        print("没有找到耗时字段")
        return
    rows.sort(key=lambda x: x[0], reverse=True)
    for value, r in rows[:args.limit]:
        print(f"{value:9.2f} {r['tag']:7s} {r['event']:20s} {r['msg']}")


def cmd_timeline(records, args):
    keep = {
        "state", "mission_done", "mission_abandoned", "goal_degraded", "goal_truncated",
        "goal_degrade_rollback", "stuck_retry", "backup_start", "backup_done",
        "backup_short", "replan_failed", "search_failed",
        "path_cycle", "path_too_long", "memory_error", "emergency_stop_failed",
    }
    for r in records:
        if r["event"] in keep:
            print_record(r)


def cmd_export(records, args):
    out = args.out or "run.jsonl"
    with open(out, "w", encoding="utf-8") as fh:
        for r in records:
            fh.write(json.dumps(r, ensure_ascii=False) + "\n")
    print(f"wrote {len(records)} records -> {out}")


def normalize_argv(argv):
    if not argv or argv[0] in COMMANDS:
        return argv
    try:
        idx = next(i for i, a in enumerate(argv) if a in COMMANDS)
    except StopIteration:
        return argv
    cmd = argv[idx]
    rest = argv[:idx] + argv[idx + 1:]
    if cmd == "rid":
        num = next((x for x in rest if re.fullmatch(r"-?\d+", x)), None)
        if num is not None:
            rest.remove(num)
            return ["rid", num] + rest
    return [cmd] + rest


def main():
    parser = argparse.ArgumentParser(description="查询/统计 SCAN-Planner 日志")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("summary")
    p.add_argument("paths", nargs="*")
    p.set_defaults(func=cmd_summary)

    p = sub.add_parser("rid")
    p.add_argument("rid", type=int)
    p.add_argument("paths", nargs="*")
    p.set_defaults(func=cmd_rid)

    p = sub.add_parser("last-failure")
    p.add_argument("paths", nargs="*")
    p.set_defaults(func=cmd_last_failure)

    p = sub.add_parser("errors")
    p.add_argument("paths", nargs="*")
    p.add_argument("--limit", type=int, default=50)
    p.set_defaults(func=cmd_errors)

    p = sub.add_parser("slow")
    p.add_argument("paths", nargs="*")
    p.add_argument("--limit", type=int, default=20)
    p.set_defaults(func=cmd_slow)

    p = sub.add_parser("timeline")
    p.add_argument("paths", nargs="*")
    p.set_defaults(func=cmd_timeline)

    p = sub.add_parser("export")
    p.add_argument("paths", nargs="*")
    p.add_argument("--out", default="run.jsonl")
    p.set_defaults(func=cmd_export)

    args = parser.parse_args(normalize_argv(sys.argv[1:]))
    records = load(args.paths)
    if not records:
        print("没有找到日志文件（可显式传入路径，或先运行 record_run.sh）", file=sys.stderr)
        return 2
    args.func(records, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
