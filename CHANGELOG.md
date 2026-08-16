# Changelog

本文件只记**发版级**的变化；每个阶段的拆片、判据与实测数据在 [tasks/plan.md](tasks/plan.md)，
决策在 [SPEC.md §9](SPEC.md) 与 [docs/adr/](docs/adr/)。

## v1.0.0 — 2026-08-16

第一版：路线图 P0a → P9 全部完成，两个仿真环境跑同一套栈、过同一张判据表。

**能力**
- 地图与路由（OpenDRIVE 解析、车道级有向图、Dijkstra）；定位（ESKF + NDT，横向 0.09–0.11 m）；
  感知（RANSAC 地面 + 欧式聚类 + L-Shape + 恒速 KF 多目标跟踪，两道物理先验准入门）；
  预测（恒速 / 车道跟随 / 不确定椭圆，6 s 视界）；行为决策（行为树：跟车 / 让行 / 通行）；
  规划（Frenet 采样 + 五次多项式 + OBB 碰撞 + 速度剖面）；控制（Stanley + 速度环 PI）。
- 双仿真环境：本地 Gazebo Harmonic（日常 + CI）与云端 CARLA 0.9.16（sidecar 全中继，验收）；
  仿真数据源可插拔，桥接层做完全相同的翻译。
- 四层测试：1202 tests（L1 + L3-G 进 CI），`run_all_scenarios.sh` 一键场景表 9/9，
  异常注入清单 15 行全有自动化红绿。

**实测（节选，判据未放宽）**
- 感知 CP-P5-B：Gazebo 9/9 ×3（横向 p95 0.15、虚警 0、ID 切换 1）；CARLA 9/9 ×2（近边 0.19–0.21、横向 0.12、ID 0）。
- 行为三场景真值层 + 感知层：两环境各 6/6；S04 绕障 9/9 + 停车 4/4；S06 红绿灯 2/2（CARLA）。
- 两环境每拍耗时表全在预算内（CARLA perception p95 8.9 ms）。

**已知边界**：仿真栈未上实车；无倒车、无相机；CARLA 侧 S01 四条跟踪判据入表放行
（[ADR-0002](docs/adr/0002-carla-s01-tracking-gap-not-fixed.md)）；定位未在 CARLA 上验收。
