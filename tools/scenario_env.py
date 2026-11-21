#!/usr/bin/env python3
# ============================================================================
# 文件：tools/scenario_env.py —— 把场景 YAML 转成 shell 变量
#
# sim_check.sh 用：
#   eval "$(python3 tools/scenario_env.py tools/scenarios/empty_3m.yaml)"
#
# 场景文件是仓库内的可信配置，值都用 shlex.quote 转义。
# ============================================================================
import shlex
import sys

import yaml

DEFAULTS = {
    "name": "custom",
    "path_file": "",
    "navi_mode": 3,
    "obstacle_number": 0,
    "use_pcd_map": False,
    "pcd_map_file": "",
    "timeout": 45,
    "min_move": 1.0,
    "goal_x": "",
    "goal_y": "",
    "goal_tol": 0.5,
    "goal_settle": 1.0,
    "obstacle_bbox": "",
    "obstacle_bboxes": [],
    "min_clearance": 0.2,
    "min_yaw_change": 0.0,
    "map_size_x": "",
    "map_size_y": "",
    "map_size_z": "",
    # ---- 目标降级断言（可选）----
    # expect_degrade: 本次运行必须出现"目标不可达→降级"的证据
    "expect_degrade": False,
    # goal_to_obstacle: 原目标点到最近障碍表面的距离（米）。
    #   用于把"停点应紧贴障碍外侧"表达成区间断言。
    "goal_to_obstacle": "",
    # 允许的停点距离区间；缺省时按 [goal_to_obstacle, goal_to_obstacle + 1.0] 推
    "goal_dist_min": "",
    "goal_dist_max": "",
    # 同一目标最多允许降级几次（幂等性：正常应为 1）
    "max_degrade_count": 1,
}

VARIABLES = {
    "name": "SCENARIO_NAME",
    "path_file": "PATH_FILE",
    "navi_mode": "NAVI_MODE",
    "obstacle_number": "OBSTACLE_NUMBER",
    "use_pcd_map": "USE_PCD_MAP",
    "pcd_map_file": "PCD_MAP_FILE",
    "timeout": "TIMEOUT",
    "min_move": "MIN_MOVE",
    "goal_x": "GOAL_X",
    "goal_y": "GOAL_Y",
    "goal_tol": "GOAL_TOL",
    "goal_settle": "GOAL_SETTLE",
    "obstacle_bbox": "OBSTACLE_BBOX",
    "obstacle_bboxes": "OBSTACLE_BBOXES",
    "min_clearance": "MIN_CLEARANCE",
    "min_yaw_change": "MIN_YAW_CHANGE",
    "map_size_x": "MAP_SIZE_X",
    "map_size_y": "MAP_SIZE_Y",
    "map_size_z": "MAP_SIZE_Z",
    "expect_degrade": "EXPECT_DEGRADE",
    "goal_to_obstacle": "GOAL_TO_OBSTACLE",
    "goal_dist_min": "GOAL_DIST_MIN",
    "goal_dist_max": "GOAL_DIST_MAX",
    "max_degrade_count": "MAX_DEGRADE_COUNT",
}


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: scenario_env.py <scenario.yaml>")
    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    for key, variable in VARIABLES.items():
        value = data.get(key, DEFAULTS[key])
        if isinstance(value, bool):
            value = "true" if value else "false"
        elif key == "obstacle_bboxes" and isinstance(value, (list, tuple)):
            value = ";".join(",".join(str(item) for item in box) for box in value)
        elif isinstance(value, (list, tuple)):
            value = ",".join(str(item) for item in value)
        print(f"{variable}={shlex.quote(str(value))}")


if __name__ == "__main__":
    main()
