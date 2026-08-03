# P0b 方案 B：最小对齐验证（上机手册）

| | |
|---|---|
| **目的** | 回答**一个**问题：CARLA 的转向执行机构响应时间 τ 是多少 |
| **不做什么** | 不建 `carla_bridge`、不接 ROS 话题、不建完整云端环境 —— 那些是方案 A |
| **决定于** | 2026-08-03，用户在 P2-S5 的三方案里选了 B |
| **产出** | 一个数字 + 一个判断：现在这组 `k_e = 1.0` 在 CARLA 上还成不成立 |

---

## 0. 为什么只量这一个数

P2 把「行为漂移」从抽象风险变成了一个具体倍数。Gazebo 的转向执行机构响应时间
原本是 1.198 s —— **纯粹因为 SDF 里从没设过 `AckermannSteering` 的转向 P 增益**、
用了插件默认值。修成 0.294 s 之后：

| | τ = 1.198 s | τ = 0.294 s | 倍数 |
|---|---|---|---|
| CP-P2-B 最大横向误差 | 0.801 m | 0.0633 m | **12.7×** |
| 横向误差 RMS | 0.252 m | 0.0159 m | **15.8×** |

同一个控制器、同一组增益、同一条路径，**只因为被控对象的一个时间常数变了**。

更要命的是它的**症状**：τ = 1.198 时车照样跑完全程、照样到达终点，只是弯道上
误差大 12.7 倍。如果这发生在 CARLA 上而我们没有基线，第一反应必然是去调 `k_e` ——
**而那是错的，被控对象才是变量。**

所以 B 要买的就是这一个数：**把那个未知量变成已知量。**

---

## 1. 上机前（本地，已完成）

- [x] `scripts/carla_align_vehicle.py` —— 对齐 + 量 τ，一个脚本跑完
- [x] `--dry-run` 已验过参数映射与单位换算（**最容易错、最不该在计费机器上调试的部分**）
- [x] `config/vehicle_params.yaml` 已含 `actuator.steer_response_time_s`（物理量，不是仿真器特有的增益）
- [x] Gazebo 侧基线：τ = **0.294 s**，稳态达成率 **100.7%**（`scripts/probe_steering_response.py` 实测）

> **为什么本机跑不了**：CARLA 强制要求 Vulkan，而本机只有 `llvmpipe`（CPU 软件光栅化）。
> `verify_gpu.sh` 全过说明的是 **OpenGL** 硬件加速（Mesa d3d12 → `/dev/dxg`），
> Vulkan 在 WSL 下要靠 `dzn`，没装。详见 `CLAUDE.md` 环境陷阱表。
> **这正是 SPEC §4.1 把环境 B 放在云端的直接原因** —— 现在有实测依据了。

---

## 2. 上机流程

### 2.1 **先验 Vulkan，再拉 CARLA**（plan.md §6 第 1 步）

这个顺序是硬性的：CARLA 镜像约 20 GB，Vulkan 不通的话拉下来也是白拉。

```bash
vulkaninfo --summary | head -20
# 必须看到 deviceType = PHYSICAL_DEVICE_TYPE_DISCRETE_GPU（或 INTEGRATED）
# 出现 llvmpipe = 软件光栅化 = 这台机器不能用，**立刻换机器，别往下走**
```

没有 `vulkaninfo` 时的零安装验法（与本地用的同一招）：

```python
python3 -c "
import ctypes
vk = ctypes.CDLL('libvulkan.so.1')
# ... 见 CLAUDE.md 里那段 ctypes 探针
"
```

### 2.2 装 CARLA 0.9.16

```bash
# 官方预编译包（约 20 GB）
wget https://tiny.carla.org/carla-0-9-16-linux -O CARLA_0.9.16.tar.gz
tar xzf CARLA_0.9.16.tar.gz && cd CARLA_0.9.16

# PythonAPI（官方提供 3.10/3.11/3.12 wheel —— 这正是 SPEC §9 D3 换到
# Jazzy/24.04 的依据之一：0.9.15 的 Python 限制已不复存在）
pip install PythonAPI/carla/dist/carla-0.9.16-cp312-*.whl
```

### 2.3 起 CARLA（无头、低画质 —— 我们只量物理，不看画面）

```bash
./CarlaUE4.sh -RenderOffScreen -quality-level=Low -carla-rpc-port=2000
```

### 2.4 跑对齐与测量

```bash
python3 scripts/carla_align_vehicle.py --host 127.0.0.1 --step-rad 0.30 --speed-mps 4.0
```

**与 Gazebo 侧用的是同一个工况**（阶跃 0.30 rad、恒速 4.0 m/s），
这样两个数才可比 —— 换工况就得两边都换。

---

## 3. 要记下来的东西

脚本会打印，但**手动抄进 `tasks/todo.md`**，因为云机器随时会被回收：

| 项 | Gazebo 基线 | CARLA 实测 |
|---|---|---|
| 63% 上升时间 τ | 0.294 s | ？ |
| 稳态达成率 | 100.7% | ？ |
| 蓝图实测轴距 | 2.700 m（生成物，精确） | ？ |
| 用的蓝图 | — | ？ |

---

## 4. 判据与后续

| τ_CARLA / τ_Gazebo | 结论 | 下一步 |
|---|---|---|
| **0.5 – 2.0×** | 同一量级，`k_e = 1.0` 大概率仍成立 | P0b 紧迫性**正式下调**（有依据不是侥幸），安心进 P3 |
| **超出该范围** | 不同量级，增益不能直接搬 | 记下差异；P3 之前决定是补 A 还是把控制参数做成按环境分档 |

**无论哪种结果，B 都算成功** —— 它买的是"知道差多少"，不是"差得少"。

---

## 5. 已知对不齐的量（**这是 B 的产出之一，不是缺陷**）

| 量 | 能否用 `apply_physics_control` 对齐 | 说明 |
|---|---|---|
| 质量、质心、轮半径、最大转角 | ✅ | 脚本已处理 |
| **轴距 / 轮距** | ❌ | 由蓝图的网格与车轮位置决定。脚本会**量出来**并报告偏差。差太多的对齐手段是**换蓝图**，不是改参数 —— 轴距直接进 Stanley 的前轴换算和自行车模型 |
| **轮胎侧偏** | ❌ | CARLA 有侧偏刚度，Gazebo 的 `AckermannSteering` 没有。这不是"要对齐的参数"，是**两个环境的本质差异** —— 也正是 CP-P2-B 之外还需要云端验收的原因（SPEC §4.1） |

---

## 6. 两个坑，别踩

**① `carla.VehicleControl.steer` 是归一化 [-1, 1]，不是弧度。**
实际转角 = `steer × wheel.max_steer_angle`。直接把 0.3 填进去，
若蓝图默认 `max_steer_angle` 是 70°（CARLA 常见），实际转的是 0.37 rad ——
差 22%，**而且不会报错**，只会让 τ 和达成率都偏。
脚本已做换算，但**改脚本时别把这条弄丢**。

**② 必须开同步模式 + 固定步长。** 异步模式下每次量出来的 τ 都不一样，
那就没法和 Gazebo 的 0.294 s 比。脚本已设 `synchronous_mode=True` +
`fixed_delta_seconds`，并在退出时恢复原设置。

---

## 7. ⚠️ 这个脚本从未在真实 CARLA 上跑过

写它的时候手边没有 Vulkan 硬件。`--dry-run` 覆盖的是**参数映射和单位换算**
—— 那是最容易错、又最不该在计费机器上调试的部分。
**连接、spawn、同步模式、`get_physics_control()` 的字段名**（CARLA 0.9.15+ 改过
若干 wheel 字段）只能上机验。**第一次上机请留出排错时间，别按"跑一条命令就完"计划。**
