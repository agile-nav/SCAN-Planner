"""机型档案完整性校验。

目的：把“三机型能不能跑”的前提固定下来。任何档案漏字段、几何不自洽、
或者 A2 的实机定稿值被误改，这个测试都会直接失败。

跑法：colcon test --packages-select scan_planner --ctest-args -R robot_profiles
"""

import math
import os
import re

import pytest
import yaml

PROFILE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "robots"))
COMMON_YAML = os.path.abspath(os.path.join(PROFILE_DIR, "..", "common.yaml"))
TEMPLATE_YAML = os.path.join(PROFILE_DIR, "_template.yaml.example")
REPO_ROOT = os.path.abspath(os.path.join(PROFILE_DIR, "..", "..", "..", "..", ".."))
ROBOTS = ("a2", "go2", "go2w")

REQUIRED_ROBOT_META = ("name", "description_package", "model_file", "root_link")
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
    """common.yaml 的通用默认值。档案里没写的键按它解析（launch 也是这个叠加顺序）。"""
    with open(COMMON_YAML, "r", encoding="utf-8") as handle:
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
def test_robot_metadata_and_model_file(robot):
    """机型元数据必须齐全，且模型文件确实存在（launch 用它生成 robot_description）。"""
    profile = load_profile(robot)
    meta = profile.get("robot")
    assert isinstance(meta, dict), f"{robot}: 缺少 robot 元数据段"
    missing = [key for key in REQUIRED_ROBOT_META if not meta.get(key)]
    assert not missing, f"{robot}: robot 元数据缺字段: {missing}"
    assert meta["name"] == robot
    assert meta["root_link"] == "base"

    model = os.path.join(REPO_ROOT, "src", "simulator", "Utils",
                         meta["description_package"], meta["model_file"])
    assert os.path.isfile(model), f"{robot}: 模型文件不存在: {model}"


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
def test_profiles_do_not_hardcode_navi_mode(robot):
    """navi_mode 是本次运行模式（launch 参数），不是机型属性。"""
    assert "fsm.navi_mode" not in params_of(robot)


@pytest.mark.parametrize("robot", ROBOTS)
def test_planning_horizon_fits_local_map(robot):
    """前视距离必须落在局部地图可信范围内。

    否则前视点会落到图外，getInflateOccupancy 返回 -1（被当作障碍），
    每轮规划都要走“局部目标兜底”，既浪费又容易选到过近的目标。
    """
    params = effective_params(robot)
    horizon = float(params["fsm.planning_horizon"])
    manager_horizon = float(params.get("manager.planning_horizon", horizon))
    radius = float(params["grid_map.double_cylinder_radius"])
    range_x = float(params["grid_map.local_update_range_x"])
    range_y = float(params["grid_map.local_update_range_y"])
    size_x = float(params["grid_map.sliding_map_size_x"])
    size_y = float(params["grid_map.sliding_map_size_y"])

    assert 0.0 < horizon <= min(range_x, range_y) - radius + 1e-9, (
        f"{robot}: planning_horizon={horizon} 超出局部更新范围 "
        f"(range={min(range_x, range_y)}, radius={radius})")
    assert 0.0 < manager_horizon <= min(range_x, range_y) - radius + 1e-9, (
        f"{robot}: manager.planning_horizon={manager_horizon} 超出局部更新范围 "
        f"(range={min(range_x, range_y)}, radius={radius})")
    assert range_x <= size_x / 2.0 + 1e-9 and range_y <= size_y / 2.0 + 1e-9, (
        f"{robot}: local_update_range 不能超过滑动地图半宽")


REQUIRED_COMMON_SECTIONS = (
    "scan_planner_node",
    "closed_loop_controller",
    "open_loop_controller",
    "quadruped_kinematic_sim",
    "quadruped_gait_publisher",
    "mockamap_node",
    "pcl_render_node",
    "odom_visualization",
)


def common_node_params(node_name):
    with open(COMMON_YAML, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data.get(node_name, {}).get("ros__parameters", {}) or {}


def test_common_has_required_node_sections():
    """common.yaml 必须包含所有节点的默认段，避免合并时漏节。"""
    with open(COMMON_YAML, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    missing = [name for name in REQUIRED_COMMON_SECTIONS if name not in data]
    assert not missing, f"common.yaml 缺少节点段: {missing}"


@pytest.mark.parametrize("robot", ROBOTS)
def test_effective_sensor_params(robot):
    """外参/深度内参必须存在、长度正确且为有限值。"""
    params = effective_params(robot)
    for key in ("grid_map.lidar_extrinsic_xyz", "grid_map.lidar_extrinsic_rpy",
                "grid_map.depth_extrinsic_xyz", "grid_map.depth_extrinsic_rpy"):
        assert key in params, f"{robot}: 缺少 {key}"
        assert len(params[key]) == 3, f"{robot}: {key} 必须是 3 元数组"
        assert all(math.isfinite(float(v)) for v in params[key])
    for key in ("grid_map.fx", "grid_map.fy", "grid_map.cx", "grid_map.cy"):
        assert key in params, f"{robot}: 缺少深度内参 {key}"
        assert math.isfinite(float(params[key]))
    assert float(params["grid_map.fx"]) > 0.0
    assert float(params["grid_map.fy"]) > 0.0


@pytest.mark.parametrize("robot", ROBOTS)
def test_robot_topics_init_shape(robot):
    init = topics_of(robot).get("init")
    if init is not None:
        assert len(init) == 3, f"{robot}: robot_topics.init 必须是 [x, y, z]"


@pytest.mark.parametrize("robot", ROBOTS)
def test_robot_model_args_is_list(robot):
    meta = load_profile(robot).get("robot", {}) or {}
    assert isinstance(meta.get("model_args", []), list)


def test_common_sensor_mount_present():
    params = common_node_params("pcl_render_node")
    for key in ("sensor_mount_xyz", "sensor_mount_rpy"):
        assert key in params, f"common.yaml pcl_render_node 缺少 {key}"
        assert len(params[key]) == 3


@pytest.mark.parametrize("robot", ROBOTS)
def test_gazebo_metadata_if_present(robot):
    """gazebo: 段是可选能力；写了就必须指向真实存在的资产。"""
    profile = load_profile(robot)
    gazebo = profile.get("gazebo")
    if gazebo is None:
        return
    for key in ("world", "bridge_config", "controllers", "spawn_name", "xacro_args"):
        assert key in gazebo, f"{robot}: gazebo 缺少 {key}"
    assert isinstance(gazebo["controllers"], list)
    assert isinstance(gazebo["xacro_args"], list)
    meta = profile["robot"]
    base = os.path.join(REPO_ROOT, "src", "simulator", "Utils",
                        meta["description_package"])
    for key in ("world", "bridge_config"):
        assert os.path.isfile(os.path.join(base, gazebo[key])), (
            f"{robot}: gazebo 资产不存在: {gazebo[key]}")


def test_go2w_official_urdf_has_wheels():
    """Go2-W 必须是官方 16 自由度模型：12 腿 + 4 轮。"""
    profile = load_profile("go2w")
    meta = profile["robot"]
    path = os.path.join(REPO_ROOT, "src", "simulator", "Utils",
                        meta["description_package"], meta["model_file"])
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    assert '<robot name="go2w">' in text
    legs = re.findall(
        r'<joint name="[A-Z]{2}_(?:hip|thigh|calf)_joint" type="revolute"', text)
    wheels = re.findall(r'<joint name="[A-Z]{2}_foot_joint" type="continuous"', text)
    assert len(legs) == 12, f"Go2-W 腿关节数不是 12: {len(legs)}"
    assert len(wheels) == 4, f"Go2-W 轮关节数不是 4: {len(wheels)}"


def test_go2w_wheel_joint_names_match_urdf():
    """档案里给步态节点的轮关节名必须和官方 URDF 一致。"""
    profile = load_profile("go2w")
    meta = profile["robot"]
    path = os.path.join(REPO_ROOT, "src", "simulator", "Utils",
                        meta["description_package"], meta["model_file"])
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    urdf_wheels = set(re.findall(
        r'<joint name="([A-Z]{2}_foot_joint)" type="continuous"', text))
    params = profile["quadruped_gait_publisher"]["ros__parameters"]
    assert set(params["wheel_joint_names"]) == urdf_wheels


def test_template_profile_is_complete():
    """新增机型模板必须覆盖 robot / 规划器 / 真机话题三部分。"""
    assert os.path.isfile(TEMPLATE_YAML), "缺少新增机型模板 _template.yaml.example"
    with open(TEMPLATE_YAML, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    assert data["robot"]["name"] == "myrobot"
    params = data["scan_planner_node"]["ros__parameters"]
    assert "fsm.navi_mode" not in params, "模板不应写死 fsm.navi_mode"
    for key in ("fsm.planning_horizon", "manager.planning_horizon",
                "grid_map.double_cylinder_radius",
                "grid_map.double_cylinder_offset", "grid_map.body_height",
                "grid_map.self_filter_radius",
                "grid_map.obstacles_inflation_z_up",
                "grid_map.obstacles_inflation_z_down"):
        assert key in params, f"模板缺少 {key}"
    topics = data["robot_topics"]
    for key in ("body_pose", "sensor_pose", "cloud", "depth", "cmd_vel",
                "motion_enabled", "cloud_is_world", "need_extrinsic"):
        assert key in topics, f"模板缺少 robot_topics.{key}"
