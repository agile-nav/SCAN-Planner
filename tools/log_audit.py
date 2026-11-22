#!/usr/bin/env python3
# ============================================================================
# 文件：tools/log_audit.py —— 日志/终端输出规范审计
#
# 用法：
#   python3 tools/log_audit.py            # 只报告，退出码 0
#   python3 tools/log_audit.py --strict   # 发现第一方裸输出/ANSI 时退出码 1
#
# 检查内容：
#   1. 第一方代码里的裸 stdout/stderr 输出（cout/cerr/printf/fprintf）；
#   2. 日志/字符串里的 ANSI 颜色转义；
#   3. 各日志级别的数量统计，便于观察 WARN/ERROR 是否失衡。
#
# 说明：
#   - 第三方目录（local_sensing/include/opengl_sim.hpp、FOV_Checker、ikd-Tree 等）
#     单独统计，不算严格模式违规；
#   - 只做文本级扫描，不替代编译器和运行时验证。
# ============================================================================
import argparse
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FIRST_PARTY_DIRS = [
    os.path.join(ROOT, "src", "planner"),
    os.path.join(ROOT, "src", "simulator"),
]
THIRD_PARTY_MARKERS = (
    os.sep + "third_party" + os.sep,
    os.sep + "ikd-Tree" + os.sep,
    os.sep + "FOV_Checker" + os.sep,
    os.sep + "shader_m.h",
    os.sep + "opengl_sim.hpp",
)
SOURCE_SUFFIXES = (".cpp", ".h", ".hpp")

RAW_OUTPUT_RE = re.compile(r"\b(?:std::)?(?:cout|cerr|clog)\b|\b(?:printf|fprintf)\s*\(")
ANSI_RE = re.compile(r"\\033|\\x1b|\\e\[")
LEVEL_RE = re.compile(r"RCLCPP_([A-Z_]+)")


def strip_comments(text):
    """去掉 // 与 /* */ 注释，保留换行与行号。"""
    out = []
    i, n = 0, len(text)
    in_line = False
    in_block = False
    in_string = False
    in_char = False
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if in_line:
            if ch == "\n":
                in_line = False
                out.append("\n")
            else:
                out.append(" ")
            i += 1
            continue
        if in_block:
            if ch == "*" and nxt == "/":
                in_block = False
                out.extend("  ")
                i += 2
            else:
                out.append("\n" if ch == "\n" else " ")
                i += 1
            continue
        if in_string:
            out.append(ch)
            if ch == "\\":
                if i + 1 < n:
                    out.append(nxt)
                    i += 2
                    continue
            elif ch == '"':
                in_string = False
            i += 1
            continue
        if in_char:
            out.append(ch)
            if ch == "\\":
                if i + 1 < n:
                    out.append(nxt)
                    i += 2
                    continue
            elif ch == "'":
                in_char = False
            i += 1
            continue
        if ch == "/" and nxt == "/":
            in_line = True
            out.extend("  ")
            i += 2
            continue
        if ch == "/" and nxt == "*":
            in_block = True
            out.extend("  ")
            i += 2
            continue
        if ch == '"':
            in_string = True
        elif ch == "'":
            in_char = True
        out.append(ch)
        i += 1
    return "".join(out)


def is_third_party(path):
    return any(marker in path for marker in THIRD_PARTY_MARKERS)


def iter_sources():
    for root in FIRST_PARTY_DIRS:
        for dirpath, _, filenames in os.walk(root):
            for name in filenames:
                if name.endswith(SOURCE_SUFFIXES):
                    yield os.path.join(dirpath, name)


def audit():
    raw_first = []
    raw_third = []
    ansi = []
    levels = {}

    for path in sorted(iter_sources()):
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            text = strip_comments(handle.read())
        rel = os.path.relpath(path, ROOT)
        for line_no, line in enumerate(text.splitlines(), 1):
            if RAW_OUTPUT_RE.search(line):
                item = (rel, line_no, line.strip())
                (raw_third if is_third_party(rel) else raw_first).append(item)
            if ANSI_RE.search(line):
                ansi.append((rel, line_no, line.strip()))
        for macro in LEVEL_RE.findall(text):
            if macro in ("INFO", "WARN", "ERROR", "DEBUG", "FATAL"):
                levels[macro] = levels.get(macro, 0) + 1
            elif macro.startswith(("INFO_", "WARN_", "ERROR_", "DEBUG_")):
                levels[macro] = levels.get(macro, 0) + 1

    print("=== 第一方裸输出（严格模式违规） ===")
    for rel, line_no, line in raw_first:
        print(f"{rel}:{line_no}: {line}")
    print(f"小计: {len(raw_first)}")

    print("\n=== 第三方裸输出（仅报告） ===")
    for rel, line_no, line in raw_third:
        print(f"{rel}:{line_no}: {line}")
    print(f"小计: {len(raw_third)}")

    print("\n=== ANSI 转义 ===")
    for rel, line_no, line in ansi:
        print(f"{rel}:{line_no}: {line}")
    print(f"小计: {len(ansi)}")

    print("\n=== RCLCPP 日志级别统计 ===")
    for key in sorted(levels):
        print(f"{key:12s} {levels[key]}")

    return raw_first, ansi


def main():
    parser = argparse.ArgumentParser(description="审计日志与终端输出规范")
    parser.add_argument("--strict", action="store_true",
                        help="第一方裸输出或 ANSI 存在时返回非 0")
    args = parser.parse_args()

    raw_first, ansi = audit()
    if args.strict and (raw_first or ansi):
        print("\n[strict] 发现第一方裸输出或 ANSI 转义", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
