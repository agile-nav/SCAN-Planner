#!/usr/bin/env python3
# ============================================================================
# 文件：tools/docs_audit.py —— 文档一致性审计
#
# 检查：
#   1. 旧文件名 / 旧节点名 / 旧说法是否还出现在用户文档里；
#   2. Markdown 本地链接指向的文件是否存在；
#   3. 关键文档是否仍覆盖 sim_check / 通用 launch / CI。
#
# 用法：python3 tools/docs_audit.py
# ============================================================================
import glob
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

DOC_FILES = [
    os.path.join(ROOT, "README.md"),
    os.path.join(ROOT, "tools", "README.md"),
    os.path.join(ROOT, "src", "planner", "plan_manage", "config", "robots", "README.md"),
]
DOC_FILES += glob.glob(os.path.join(ROOT, "docs", "*.md"))
DOC_FILES += glob.glob(os.path.join(ROOT, "src", "simulator", "Utils", "*", "README.md"))

BANNED = [
    (re.compile(r"(?<![\w/])(?:planner|controllers|simulator)\.yaml"), "旧配置文件"),
    (re.compile(r"_ROBOT_MODELS"), "已删除的机型表"),
    (re.compile(r"\bgo2_kinematic_sim\b"), "旧节点名"),
    (re.compile(r"\bgo2_gait_publisher\b"), "旧节点名"),
    (re.compile(r"\ba2_rviz\.launch\b"), "已删除的 launch"),
    (re.compile(r"\bgo2_rviz\.launch\b"), "已删除的 launch"),
    (re.compile(r"\bgo2_sim\.launch\b"), "已删除的 launch"),
    (re.compile(r"由 Go2 派生|无官方 URDF|没有发布 Go2-W|腿长沿用 Go2"), "Go2-W 旧说法"),
    (re.compile(r"sensor2body\s*固定|lidar_pitch\s*固定"), "传感器安装旧说法"),
]

WHITELIST_LINE = ("已移除", "原 planner.yaml", "原 controllers.yaml", "原 simulator.yaml")


def iter_lines():
    for path in DOC_FILES:
        if not os.path.isfile(path):
            continue
        with open(path, "r", encoding="utf-8") as handle:
            for line_no, line in enumerate(handle, 1):
                yield path, line_no, line.rstrip("\n")


def audit_banned_text():
    errors = []
    for path, line_no, line in iter_lines():
        if any(token in line for token in WHITELIST_LINE):
            continue
        for pattern, reason in BANNED:
            if pattern.search(line):
                rel = os.path.relpath(path, ROOT)
                errors.append(f"{rel}:{line_no}: {reason}: {line.strip()}")
    return errors


def audit_local_links():
    errors = []
    link_re = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
    for path, line_no, line in iter_lines():
        for target in link_re.findall(line):
            target = target.strip().strip("<>")
            if not target or target.startswith(("http://", "https://", "mailto:", "#")):
                continue
            target = target.split("#", 1)[0].split("?", 1)[0]
            if not target:
                continue
            if target.startswith("/"):
                resolved = os.path.join(ROOT, target.lstrip("/"))
            else:
                resolved = os.path.normpath(os.path.join(os.path.dirname(path), target))
            if not os.path.exists(resolved):
                rel = os.path.relpath(path, ROOT)
                errors.append(f"{rel}:{line_no}: 链接不存在: {target}")
    return errors


def audit_required_mentions():
    errors = []
    checks = {
        os.path.join(ROOT, "tools", "README.md"): [
            "sim_check", "robot_view", "robot_gazebo", "scenario", "CI",
        ],
        os.path.join(ROOT, "src", "planner", "plan_manage", "config", "robots", "README.md"): [
            "_template.yaml.example", "robot_topics",
        ],
    }
    for path, words in checks.items():
        if not os.path.isfile(path):
            errors.append(f"缺少文档: {os.path.relpath(path, ROOT)}")
            continue
        text = open(path, "r", encoding="utf-8").read()
        for word in words:
            if word not in text:
                errors.append(f"{os.path.relpath(path, ROOT)}: 缺少关键内容 {word!r}")
    return errors


def main():
    errors = []
    errors += audit_banned_text()
    errors += audit_local_links()
    errors += audit_required_mentions()
    if errors:
        print("文档审计失败：")
        for error in errors:
            print("  -", error)
        return 1
    print(f"文档审计通过（{len(set(DOC_FILES))} 个文件）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
