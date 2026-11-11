# robot_overrides —— 真机上"人工改动过"的配置快照

`~/nav_ws` 不是 git 仓库（只有 `global_planner` 包自身是，且它相对 HEAD 是整份重写），
所以这些**直接改在真机上**的参数没法用 diff 追踪。这里保存它们的当前内容，
用于：① 记录每次人工改动；② 下次改动时可以直接 diff；③ 回退有据可查。

| 文件 | 真机路径 |
|---|---|
| `global_planner/global_planner.yaml` | `~/nav_ws/src/global_planner/config/global_planner.yaml` |
| `super_lio/map_localization.yaml`  | `~/nav_ws/src/super_lio/super_lio/config/map_localization.yaml` |

## 2026-11-10 20:09 / 20:11 的人工改动（本次记录）

1. `global_planner.yaml`
   - **`path_z_offset: 0.5 → 0.7`**
     ⇒ 全局路径的 z 被抬高 0.7 m。`navi_mode=3` 下局部规划器**直接使用全局路径的 z**
     作为局部目标 ⇒ 局部目标系统性偏高 0.7 m，规划出的轨迹带固定爬升，
      与实测"失败段 |Δz| 中位 0.5~1.0 m"吻合（`dyn_a_star` 的搜索平面因此常年是斜的）。
   - 另有 `clearance_distance` / `clearance_weight` 两行在当前文件中不存在（旧配置有）。
2. `map_localization.yaml`
   - **`lio.sensor.blind: 0.3`**（另一个 bag 档是 2.0，二者雷达不同，仅作参照）
     ⇒ LIO 丢弃传感器 0.3 m 内的点；与建图侧 `grid_map.self_filter_radius: 0.45` 叠加，
     机器人脚下 0.45 m 内基本无点。
   - 其它与 bag 档的差异（lidar_type / maxrange / filter_rate / topic）属"真机 vs bag"固有区别。
