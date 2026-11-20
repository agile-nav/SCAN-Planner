# ============================================================================
# 文件：launch/robot_profile.py —— 各 launch 共用的机型档案工具
#
# 目的：run / robot_view / robot_gazebo 不再各自维护一份“读档案、拼
# robot_description、取节点参数”的逻辑。
# ============================================================================
"""Shared helpers for loading robot profiles and building model commands."""

import os

import yaml

from ament_index_python.packages import get_package_share_directory


FALLBACK_ROBOT_META = {
    "name": "go2",
    "description_package": "go2_description",
    "model_file": "xacro/robot.xacro",
    "model_args": ["use_gazebo:=false"],
    "root_link": "base",
}


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def profile_path(scan_share, robot):
    return os.path.join(scan_share, "config", "robots", robot + ".yaml")


def load_robot_profile(scan_share, robot):
    path = profile_path(scan_share, robot)
    if not os.path.isfile(path):
        profile_dir = os.path.dirname(path)
        available = []
        if os.path.isdir(profile_dir):
            available = sorted(name[:-5] for name in os.listdir(profile_dir)
                               if name.endswith(".yaml"))
        raise RuntimeError(
            "unknown robot profile '%s'; available profiles: %s"
            % (robot, ", ".join(available) if available else "(none)"))
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_profile_file(path):
    if not path or not os.path.isfile(path):
        raise RuntimeError("robot profile file not found: %s" % path)
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def profile_params(profile, node_name):
    return dict(profile.get(node_name, {}).get("ros__parameters", {}) or {})


def description_path(meta, relative):
    package = meta["description_package"]
    return os.path.join(get_package_share_directory(package),
                        *str(relative).split("/"))


def robot_description_command(robot_meta, extra_args=None):
    """Build the xacro command for a model entry (pure URDF also works)."""
    meta = robot_meta or FALLBACK_ROBOT_META
    model = description_path(meta, meta["model_file"])
    args = list(meta.get("model_args", [])) if extra_args is None else list(extra_args)
    extra = " ".join(str(arg) for arg in args)
    return "xacro " + model + (" " + extra if extra else "")
