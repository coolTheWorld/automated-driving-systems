# 实施计划

> 依据：[SPEC.md](../SPEC.md)　|　创建：2026-07-30　|　当前阶段：**P0a**
> 本文件说明**怎么拆、为什么这么拆**；可执行清单见 [todo.md](./todo.md)。

---

## 0. 本次规划的范围

| 阶段 | 规划粒度 |
|------|---------|
| **P0a 本地 Gazebo 环境** | **任务级详拆**（本文件主体） |
| P0b 云端 CARLA 环境 | 轮廓级 |
| P1-P9 | 见 SPEC §10，到时再拆 |

**为什么不一次拆完**：P0a 会暴露大量环境层面的真实约束（容器内 GPU 行为、
Gazebo 实际帧率、ros_gz 的坑）。现在详拆 P3 的规划任务，等做到那里一定要重写。
**计划的有效期不超过下一个反馈点。**

---

## 1. 规划前已核实的事实

不凭记忆下结论，以下均已联网查证（2026-07-30）：

| 事实 | 结论 | 影响 |
|------|------|------|
| **Jazzy + Gazebo Harmonic** | **官方组合**，`ros-jazzy-ros-gz` 直接可用 | 无版本冲突问题，S2 大幅简化 |
| Gazebo Fortress EOL | **2026-09**（约一个月后） | 确认不选 Fortress |
| Gazebo Harmonic 支持期 | 至 **2028-09** | 覆盖项目周期 |
| Jazzy 支持期 | 至 **2029-05** | 覆盖项目周期（Humble 只到 2027-05） |
| CARLA 0.9.16 Python wheel | 官方提供 **3.10 / 3.11 / 3.12** | Ubuntu 24.04 (Python 3.12) 可直接用 |
| CARLA 0.9.16 原生 ROS 2 | `--ros2` 内置 Fast DDS，**无需 ros-bridge** | 但功能不全，缺 TF 树/ego 状态，见 SPEC §4.1 |

> **版本决策已于 2026-07-30 修订**：由 Humble + Ubuntu 22.04 + CARLA 0.9.15
> 改为 **Jazzy + Ubuntu 24.04 + CARLA 0.9.16**，理由见 SPEC §9 D3。
> 本计划已按新版本重写。

---

## 2. 依赖图

```
                    ┌──────────────────────────────┐
                    │  S1  容器 + GPU + GUI         │  ← 最高风险，先做
                    │      (go / no-go 检查点)      │
                    └───────────────┬──────────────┘
                                    │ 硬件加速确认可用
                    ┌───────────────▼──────────────┐
                    │  S2  Gazebo 世界 + 车能动     │
                    │      (仅用 gz 原生话题)       │
                    └───────────────┬──────────────┘
                                    │ 仿真本身可用
                    ┌───────────────▼──────────────┐
                    │  S3  ROS 2 桥接 + 规范话题    │
                    │      + RViz2 看到点云         │
                    └───────┬───────────────┬──────┘
                            │               │
            ┌───────────────▼──────┐   ┌────▼──────────────────┐
            │  S4  键盘 teleop 闭环 │   │  S5  工程化收尾        │
            │      车能开起来       │   │  CI / lint / 参数单源  │
            └───────────────┬──────┘   └────┬──────────────────┘
                            └───────┬───────┘
                                    ▼
                            P0a 完成 → P1 或 P0b
```

**S1→S2→S3 必须串行**，因为每一步都依赖前一步的能力被证实。
**S4 与 S5 可并行**。

---

## 3. 为什么这样切片

### 垂直切，不横着切

一个常见的错误拆法是按组件横切：「先写完所有 Dockerfile」→「再写完所有 SDF」→
「再写完所有 ROS 节点」。这样做的问题是**直到最后一步才知道能不能跑通**，
中间积累的假设全部未经验证。

本计划按**能力**竖着切：每个切片结束时，系统都比上一个切片多具备一项
**可以亲眼验证**的能力。

| 切片 | 结束时新增的能力 |
|------|----------------|
| S1 | 「我能在容器里跑起带图形界面的程序，而且是硬件加速的」 |
| S2 | 「我有一个能跑的仿真世界，里面的车能动」 |
| S3 | 「仿真数据以我定义的话题格式进了 ROS，RViz 能看见」 |
| S4 | 「我能用键盘把车开起来」——闭环成立 |
| S5 | 「这套东西别人 clone 下来能复现，改坏了 CI 会拦住」 |

### S1 必须是第一个，而且是 go/no-go 点

整个环境 A 方案建立在一个假设上：**容器内能拿到 WSL2 的 OpenGL 硬件加速**。

宿主机上已实测可用（SPEC §4 有 `glxinfo` 输出），但**容器内是另一回事**——
需要 `/dev/dri` 正确映射、容器内 Mesa 版本带 d3d12 驱动、WSLg 的 X11 socket 挂进去。
任何一环断了，Gazebo 要么起不来，要么退化成软件渲染（llvmpipe，帧率个位数）。

**所以 S1 不是「搭环境」，是「验证一个决定整个方案成立与否的假设」。**
它失败的话，S2-S5 全部作废，需要回到 SPEC §4.1 重新讨论环境 A。

---

## 4. 任务详拆

### S1　容器 + GPU + GUI【go/no-go】

| # | 任务 | 验收标准 | 验证方法 |
|---|------|---------|---------|
| 1.1 | 宿主机装 Docker | `docker run hello-world` 成功 | 命令返回 0 |
| 1.2 | 写 `docker/Dockerfile`：Ubuntu 24.04 + ROS 2 Jazzy desktop + mesa 工具 | 镜像构建成功 | `docker compose build` |
| 1.3 | 写 `docker/docker-compose.local.yml`：映射 `/dev/dri`、挂载 WSLg socket、传 `DISPLAY` | 容器能启动 | `docker compose up -d` |
| 1.4 | **验证容器内硬件加速** | `glxinfo -B` 输出 `Accelerated: yes` 且 Renderer 含 `D3D12` / `Radeon` | 容器内执行 |
| 1.5 | **验证容器内 GUI** | `rviz2` 能在 Windows 桌面弹出窗口且可交互 | 肉眼确认 |

**关键实现要点**（以下为 2026-07-30 实测后的定稿，前四条缺一不可）：

- 映射 **`/dev/dxg`** —— WSL2 的 GPU 通路是 dxgkrnl 的这个字符设备
- 映射 `/dev/dri`（`card0` + `renderD128`），并以数字 GID `group_add` 注入权限
- 设 **`GALLIUM_DRIVER=d3d12`** —— Ubuntu 24.04 的 Mesa 25.2.8 不会自动选中 d3d12
- 设 `LD_LIBRARY_PATH=/usr/lib/wsl/lib` —— `d3d12_dri.so` 用 dlopen 找 `libdxcore.so`
- WSLg 图形通路：挂载 `/tmp/.X11-unix`、`/mnt/wslg`、`/usr/lib/wsl`，
  设置 `DISPLAY`、`WAYLAND_DISPLAY`、`XDG_RUNTIME_DIR`
- 容器内需装 `mesa-utils`、`libgl1-mesa-dri`（提供 `d3d12_dri.so`）

**实测结果（S1 已完成）**：`D3D12 (AMD Radeon 780M Graphics)`，`Accelerated: yes`，
OpenGL core 4.6，`rviz2` 在 Windows 桌面弹窗且交互流畅。**CP1 通过。**

> **这一步的教训值得记住。** 前两条是排查 1.4 时才发现的——初版 compose 两条都缺，
> 结果是 `glxinfo` 报 `llvmpipe` / `Accelerated: no`，**而且没有任何报错信息**。
> 如果当初图省事跳过 1.4 直接做 S2，症状会变成「Gazebo 能起但帧率个位数」，
> 那时你面对的是 SDF 世界、物理引擎、传感器插件一大堆变量，
> 根本想不到根因在两个环境变量上。
>
> **1.4 的价值不在于它「通过了」，而在于它把故障拦在了变量最少的时刻。**

---

### S2　Gazebo 世界 + 车能动

| # | 任务 | 验收标准 | 验证方法 |
|---|------|---------|---------|
| 2.1 | Dockerfile 加装 `ros-jazzy-ros-gz`（官方组合，自动拉 Harmonic） | 构建成功 | `gz sim --versions` 显示 Harmonic |
| 2.2 | 最小世界 `worlds/campus_minimal.sdf`：地面 + 一段直路 + 若干静态方块障碍 | `gz sim` 能加载 | GUI 里可见 |
| 2.3 | 车辆模型 `models/ego_vehicle/`：车体 + 四轮 + `AckermannSteering` 插件 | 车出现在世界里 | GUI 里可见 |
| 2.4 | 车辆物理参数写入 `config/vehicle_params.yaml`（**单一来源**，SPEC §4.1 强制要求） | 轴距/最大转角/质量/加减速限值齐全且带单位 | 人工检查 |
| 2.5 | 用 gz 原生话题让车动起来 | 发布指令后车前进/转向 | `gz topic -t /model/ego/cmd_vel -m ... -p ...` |
| 2.6 | **测量实时率 RTF** | **RTF ≥ 0.8** | GUI 状态栏读数 |

> **2.6 是本切片的隐藏关键**。RTF（Real Time Factor）低于 0.8 意味着仿真比真实时间慢，
> 虽然同步模式下不影响正确性，但会严重拖慢你的迭代速度。若不达标，先简化世界几何
> （减面数、关阴影），而不是急着往下做。

---

### S3　ROS 2 桥接 + 规范话题 + RViz2

| # | 任务 | 验收标准 | 验证方法 |
|---|------|---------|---------|
| 3.1 | 建 colcon 工作区骨架（`src/` 下 SPEC §5 列出的包，先建空壳） | `colcon build` 通过 | 命令返回 0 |
| 3.2 | `ads_msgs`：定义最小消息集（控制指令、障碍物） | 编译通过，`ros2 interface show` 可见 | 命令验证 |
| 3.3 | 车辆加 `gpu_lidar` 传感器（32 线，水平 1800，量程 50 m —— 按 SPEC §2 ODD） | Gazebo 内有点云 | `gz topic -e` 有数据 |
| 3.4 | `ads_simulation/gazebo_bridge`：`ros_gz_bridge` 配置 + **话题重映射到 SPEC §4.1 规范名** | `/lidar/points`、`/imu`、`/odom`、`/clock` 均存在 | `ros2 topic list` |
| 3.5 | 验证点云频率与坐标系 | `/lidar/points` **≥ 9 Hz**，frame_id = `base_link` | `ros2 topic hz` / `echo --once` |
| 3.6 | 验证仿真时间 | 所有节点 `use_sim_time=true`，`/clock` 正常推进 | `ros2 param get` |
| 3.7 | TF 树：`map` → `odom` → `base_link` | TF 树完整无断裂 | `ros2 run tf2_tools view_frames` |
| 3.8 | RViz2 配置 `ads_visualization/rviz/default.rviz` | 点云 + TF + 车模型同时正确显示 | 肉眼确认 |

> **3.4 是整个双环境方案的技术枢纽**。这里做的事是把 Gazebo 的原生话题名
> 翻译成 SPEC 定义的规范名。P0b 里 `carla_bridge` 做完全相同的翻译。
> **翻译层做对了，上游算法就永远不需要知道数据来自哪个仿真器。**

---

### S4　键盘 teleop 闭环

| # | 任务 | 验收标准 | 验证方法 |
|---|------|---------|---------|
| 4.1 | teleop 节点：键盘 → `/vehicle_cmd`（转角 rad + 加速度 m/s²） | 按键有对应消息发出 | `ros2 topic echo /vehicle_cmd` |
| 4.2 | `gazebo_bridge` 订阅 `/vehicle_cmd` → 转 Gazebo 控制指令 | 车响应键盘 | 肉眼确认 |
| 4.3 | 指令限幅：超出 `vehicle_params.yaml` 定义的范围时截断并告警 | 极限输入不会让车飞出去 | 手动发超限指令 |
| 4.4 | **闭环验证**：键盘开车绕障碍物一圈 | 全程 RViz 中点云、TF、车姿态同步更新 | 肉眼确认 |

---

### S5　工程化收尾（可与 S4 并行）

| # | 任务 | 验收标准 | 验证方法 |
|---|------|---------|---------|
| 5.1 | `.clang-format`（ROS 2 风格，SPEC §7） | `clang-format --dry-run` 无差异 | 命令验证 |
| 5.2 | `stack.launch.py`，支持 `sim:=gazebo` 参数 | 一条命令起全栈 | `ros2 launch` |
| 5.3 | L1 单元测试样板：`ads_common` 一个几何函数 + 对应 gtest | `colcon test` 通过 | 命令返回 0 |
| 5.4 | CI（GitHub Actions）：`colcon build` + `colcon test` + lint | CI 绿 | push 后看结果 |
| 5.5 | `README.md`：从零复现 P0a 的步骤 | 照着做能跑起来 | 人工走一遍 |
| 5.6 | `docs/adr/0001-dual-simulation.md`：记录双环境决策 | 与 SPEC §9 D4 一致 | 人工检查 |

> **5.4 的 CI 现在只跑 L1+L2。** L3-G 场景测试要等 P8 才建立——
> 但**基础设施现在就要搭好**，否则以后没人补。

---

## 5. 检查点

| 检查点 | 位置 | 决策内容 |
|--------|------|---------|
| **CP1** | S1 结束 | **go / no-go**：容器内硬件加速是否可用。失败则环境 A 方案重议 |
| **CP2** | S3 结束 | 数据流是否打通、频率是否达标。这是 P1 能否开始的前提 |
| **CP3** | S5 结束 | P0a 验收。决定下一步做 P1（继续本地）还是 P0b（先建云端） |

**每个检查点我会停下来汇报实测数据**，不自行决定继续。

---

## 6. P0b 轮廓（暂不详拆）

1. Vast.ai 选机（按 SPEC §4.1 硬性条件）→ **`vulkaninfo` 验证**（先验证再拉 CARLA）
2. 装 CARLA 0.9.16 + 实测同步模式帧率
3. **验证 `--ros2` 原生接口与 Jazzy 的互通性**（本阶段最大未知项）
   —— 失败则退路是单开 Humble 容器只跑 `carla_bridge`
4. `carla_bridge` sidecar：用 PythonAPI（3.12 wheel）补齐原生接口缺失的
   TF 树 / ego 状态 / 红绿灯 / 地图，翻译到与 `gazebo_bridge` **完全相同**的规范话题名
5. SSH / noVNC / 收尾脚本（git push + scp）
6. **建立两环境一致性测试**（SPEC §4.1）：相同开环指令跑 30 s，横向偏差 < 0.5 m

> P0b 可推迟到 P4 之后，但**越晚做，积累的行为漂移越难排查**。

---

## 7. 本阶段风险

| 风险 | 触发点 | 对策 |
|------|--------|------|
| ~~容器内拿不到硬件加速~~ **已消除** | S1 (1.4) | 实际触发过，根因是缺 `/dev/dxg` 映射 + 缺 `GALLIUM_DRIVER=d3d12`；已修复并加进 `verify_gpu.sh` 的自动检查 |
| 误用 CycloneDDS | S3 | Dockerfile 固定 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`——CARLA 原生 ROS 2 只支持 Fast DDS |
| RTF 过低 | S2 (2.6) | 简化世界几何、关阴影；不达标不往下做 |
| 话题命名与 P0b 不一致 | S3 (3.4) | 规范名以 SPEC §4.1 为准，`carla_bridge` 后续对齐 |

---

## 8. 给初学者的说明

**为什么花这么大力气做 P0a，一行算法都没写？**

因为自动驾驶开发里，**环境问题和算法问题混在一起是最致命的**。
如果你的环境不稳定——今天能跑明天不能、帧率忽高忽低、话题偶尔断——
那么当车开歪的时候，你根本无法判断是控制器写错了，还是仿真抽风了。

P0a 的产出不是代码，是**一个可信的基准**：从此以后车开歪了，
你可以确信问题出在你的算法里。这个确信非常值钱。
