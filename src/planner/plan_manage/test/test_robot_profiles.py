"""机型档案完整性校验。

目的：把“三机型能不能跑”的前提固定下来。任何档案漏字段、几何不自洽、
或者 A2 的实机定稿值被误改，这个测试都会直接失败。

跑法：colcon test --packages-select scan_planner --ctest-args -R robot_profiles
"""

import math
import os

import pytest
import yaml

PROFILE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "robots"))
PLANNER_YAML = os.path.abspath(os.path.join(PROFILE_DIR, "..", "planner.yaml"))
ROBOTS = ("a2", "go2", "go2w")

REQUIRED_PARAMS = (
    "grid_map.double_cylinder_radius",
    "grid_map.double_cylinder_offset",
    "grid_map.self_filter_radius",
    "grid_map.obstacles_inflation_z_up",
    "grid_map.obstacles_inflation_z_down",
    "grid_map.body_height",
    "fsm.planning_horizon",
)
REQUIRED_TOPICS = (
    "body_pose",
    "sensor_pose",
    "cloud",
    "depth",
    "cmd_vel",
    "motion_enabled",
)


def load_profile(robot):
    path = os.path.join(PROFILE_DIR, robot + ".yaml")
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def params_of(robot):
    return load_profile(robot).get("scan_planner_node", {}).get("ros__parameters", {}) or {}


def default_params():
    """planner.yaml 的通用默认值。档案里没写的键按它解析（launch 也是这个叠加顺序）。"""
    with open(PLANNER_YAML, "r", encoding="utf-8") as handle:
        params = (yaml.safe_load(handle) or {}).get("scan_planner_node", {}).get("ros__parameters", {})
    return params or {}


def effective_params(robot):
    params = dict(default_params())
    params.update(params_of(robot))
    return params


def topics_of(robot):
    return load_profile(robot).get("robot_topics", {}) or {}


@pytest.mark.parametrize("robot", ROBOTS)
def test_profile_file_exists_and_has_node_section(robot):
    profile = load_profile(robot)
    assert "scan_planner_node" in profile
    assert "ros__parameters" in profile["scan_planner_node"]


@pytest.mark.parametrize("robot", ROBOTS)
def test_required_params_present(robot):
    params = params_of(robot)
    missing = [key for key in REQUIRED_PARAMS if key not in params]
    assert not missing, f"{robot} 缺少参数: {missing}"


@pytest.mark.parametrize("robot", ROBOTS)
def test_required_topics_present(robot):
    topics = topics_of(robot)
    missing = [key for key in REQUIRED_TOPICS if not topics.get(key)]
    assert not missing, f"{robot} 缺少 robot_topics 字段: {missing}"
    assert isinstance(topics.get("cloud_is_world"), bool)
    assert isinstance(topics.get("need_extrinsic"), bool)


@pytest.mark.parametrize("robot", ROBOTS)
def test_geometry_is_consistent(robot):
    params = params_of(robot)
    radius = float(params["grid_map.double_cylinder_radius"])
    offset = float(params["grid_map.double_cylinder_offset"])
    z_up = float(params["grid_map.obstacles_inflation_z_up"])
    z_down = float(params["grid_map.obstacles_inflation_z_down"])
    self_filter = float(params["grid_map.self_filter_radius"])
    body_height = float(params["grid_map.body_height"])

    assert radius > 0.0
    assert 0.0 <= offset <= radius, f"{robot}: 双圆柱必须有实体交集（offset <= radius）"
    assert z_up >= 0.0
    assert z_down > 0.0
    assert self_filter > 0.0, f"{robot}: 真机必须配置自身点云过滤半径"
    assert body_height >= 0.0


def test_a2_uses_real_world_final_values():
    """A2 的碰撞参数是实机调试定稿值，不允许被“公式”悄悄覆盖。"""
    params = params_of("a2")
    assert float(params["grid_map.double_cylinder_radius"]) == pytest.approx(0.25, abs=1e-6)
    assert float(params["grid_map.double_cylinder_offset"]) == pytest.approx(0.18, abs=1e-6)


@pytest.mark.parametrize("robot,length,width", (("go2", 0.70, 0.31), ("go2w", 0.70, 0.43)))
def test_go2_profiles_follow_coverage_formula(robot, length, width):
    """Go2 / Go2-W 的档案按“两圆覆盖站立矩形”换算：offset=L/4, radius=hypot(L/4,W/2)。"""
    params = params_of(robot)
    assert float(params["grid_map.double_cylinder_offset"]) == pytest.approx(length / 4.0, abs=1e-3)
    assert float(params["grid_map.double_cylinder_radius"]) == pytest.approx(
        math.hypot(length / 4.0, width / 2.0), abs=2e-3)


@pytest.mark.parametrize("robot", ROBOTS)
def test_planning_horizon_fits_local_map(robot):
    """前视距离必须落在局部地图可信范围内。

    否则前视点会落到图外，getInflateOccupancy 返回 -1（被当作障碍），
    每轮规划都要走“局部目标兜底”，既浪费又容易选到过近的目标。
    """
    params = effective_params(robot)
    horizon = float(params["fsm.planning_horizon"])
    radius = float(params["grid_map.double_cylinder_radius"])
    range_x = float(params["grid_map.local_update_range_x"])
    range_y = float(params["grid_map.local_update_range_y"])
    size_x = float(params["grid_map.sliding_map_size_x"])
    size_y = float(params["grid_map.sliding_map_size_y"])

    assert 0.0 < horizon <= min(range_x, range_y) - radius + 1e-9, (
        f"{robot}: planning_horizon={horizon} 超出局部更新范围 "
        f"(range={min(range_x, range_y)}, radius={radius})")
    assert range_x <= size_x / 2.0 + 1e-9 and range_y <= size_y / 2.0 + 1e-9, (
        f"{robot}: local_update_range 不能超过滑动地图半宽")
