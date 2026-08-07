#!/usr/bin/env python3
"""从 config/vehicle_params.yaml 生成自车模型 —— Gazebo 的 SDF 和 ROS 的 URDF。

为什么要生成而不是手写
----------------------
SPEC §4.1 把「车辆参数单一来源」列为强制要求：轴距、最大转角、质量、
传感器安装位姿等必须只在 config/vehicle_params.yaml 里定义一次。

手写做不到这一点。轴距这一个数字在 SDF 里就要出现四次以上
（前后轮位置、Ackermann 插件的 wheel_base、车体盒的中心偏移），
改一处漏三处的事迟早会发生。而它的后果不是报错，是**车在两个仿真里
开出不同的轨迹**，然后你去怀疑控制器 —— SPEC §4.1 说的「行为漂移」。

为什么要同时生成 SDF 和 URDF
----------------------------
两者描述同一辆车，但服务于两套完全不同的消费者：

    model.sdf   → Gazebo 物理引擎。决定车**怎么动**（质量、惯量、摩擦、关节驱动）。
    *.urdf      → ROS 的 robot_state_publisher。决定 TF 树长什么样、RViz 里画什么。

ROS 不认 SDF，Gazebo 也不用 URDF 建物理。如果手写 URDF，轴距和传感器外参
就等于又抄了一遍 —— 症状是 **RViz 里点云和车模型对不上**，或者更隐蔽的：
TF 报的 base_link→lidar_link 与 Gazebo 里雷达的实际安装位置差了几厘米，
下游一切配准全部带着这个偏差，而没有任何报错。

所以两份都是生成物。改参数改 YAML，然后重新跑本脚本。

用法
----
    python3 scripts/gen_vehicle_model.py            # 生成
    python3 scripts/gen_vehicle_model.py --check    # 只校验是否与 YAML 同步（供 CI 用）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / "config" / "vehicle_params.yaml"
SDF_FILE = REPO_ROOT / "models" / "ego_vehicle" / "model.sdf"
# URDF 放在 ads_visualization 下：它在 P0a 的唯一消费者是 robot_state_publisher +
# RViz。等 P4/P5 有更多模块依赖机器人描述时，按 ROS 惯例应当独立成
# ads_description 包 —— 那属于改动 SPEC §5 的模块划分，要先问过再做。
URDF_FILE = REPO_ROOT / "src" / "ads_visualization" / "urdf" / "ego_vehicle.urdf"


def box_inertia(mass: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """实心长方体绕自身质心的转动惯量（主轴）。"""
    k = mass / 12.0
    return k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y)


def wheel_inertia(mass: float, radius: float, width: float) -> tuple[float, float, float]:
    """圆柱绕自身质心的转动惯量，**旋转轴沿 Y**（车轮的滚动轴）。

    Iyy 是绕滚动轴的，用 (1/2)mr²；另两轴用 (1/12)m(3r²+w²)。
    这两个值搞反的表现是车轮转起来像个陀螺，车会诡异地自己拐弯。
    """
    ixx = izz = (1.0 / 12.0) * mass * (3.0 * radius * radius + width * width)
    iyy = 0.5 * mass * radius * radius
    return ixx, iyy, izz


def derive(p: dict) -> dict:
    """由 YAML 原始参数推导出建模需要的中间量。

    SDF 和 URDF 必须调用**同一个**推导函数。若两边各算一次车体盒中心偏移，
    迟早会出现「Gazebo 里车体前移了 0.35 m 而 RViz 里没有」这种
    只能靠肉眼比对才发现的错位。
    """
    geo = p["geometry"]
    whl = p["wheel"]

    length = geo["length_m"]
    height = geo["height_m"]
    clearance = geo["ground_clearance_m"]
    track = geo["track_width_m"]

    # ---- 坐标原点的选择 ----
    # base_link 放在**后轴中心、地面高度**（Autoware 惯例）。
    # 自行车模型、Stanley、纯追踪都以后轴为参考点推导，原点选这里能让
    # 控制算法直接使用位姿，不必到处做偏移换算 —— 偏移换算写错是常见 bug。
    chassis_len_z = height - clearance
    chassis_center_x = length / 2.0 - geo["rear_overhang_m"]  # 车体盒中心相对后轴的纵向偏移
    chassis_center_z = clearance + chassis_len_z / 2.0

    # 车体盒宽度取「轮距 - 轮宽」，正好让车轮内侧与车体侧面相切。
    # 直接用 width_m(1.8) 会把车轮整个包进车体里 —— 物理上无害（同一模型内
    # 各 link 默认不自碰撞），但看上去像块砖，调试转向时根本看不出前轮转了没有。
    # 整车外廓宽度 width_m 是**含轮胎的保守包络**，供规划做碰撞检查，不用于建模。
    chassis_width = track - whl["width_m"]

    chassis_mass = p["mass"]["total_kg"] - 4.0 * whl["mass_kg"]

    d = {
        "wheelbase": geo["wheelbase_m"],
        "track": track,
        "half_track": track / 2.0,
        "length": length,
        "height": height,
        "com_h": geo["com_height_m"],
        "clearance": clearance,
        "wheel_r": whl["radius_m"],
        "wheel_w": whl["width_m"],
        "wheel_m": whl["mass_kg"],
        "chassis_mass": chassis_mass,
        "chassis_len_z": chassis_len_z,
        "chassis_center_x": chassis_center_x,
        "chassis_center_z": chassis_center_z,
        "chassis_width": chassis_width,
        "steer_limit": p["limits"]["max_steer_angle_rad"],
        # AckermannSteering 对转向关节用**速度控制**：关节速度 = gain × 角度误差。
        # 那就是一个一阶环节，时间常数恰为 1/gain。所以把 YAML 里的**物理量**
        # （响应时间常数）换算成插件要的**实现量**（P 增益）就是取倒数。
        #
        # 做成推导量而不是让人直接填 gain，理由同 gen_map.py 里的 cutback：
        # 增益是求解器的实现细节，换个仿真器就没有对应物；
        # 而"给一个转角阶跃，多久转到 63%"是能拿去和 CARLA / 真车对齐的物理量。
        # 让人填 gain 等于把一个只有 Gazebo 认识的数字放进双环境的单一来源里。
        "steer_p_gain": 1.0 / p["actuator"]["steer_response_time_s"],
    }
    d["c_ixx"], d["c_iyy"], d["c_izz"] = box_inertia(
        chassis_mass, length, chassis_width, chassis_len_z)
    d["w_ixx"], d["w_iyy"], d["w_izz"] = wheel_inertia(
        whl["mass_kg"], whl["radius_m"], whl["width_m"])
    return d


def _imu_axis_noise(n: dict, kind: str, unit: str, indent: int) -> str:
    """三个轴各一份相同的噪声块（IMU 的 x/y/z 在 SDF 里必须分别写）。

    三轴给同一组参数是有意的简化：真实器件三轴略有差异，但那个差异
    对 ESKF 的结构没有任何影响 —— 它本来就按轴独立建模。
    让三轴不同只会让 YAML 多出六个没人知道怎么定的数。
    """
    pad = " " * indent
    body = (
        f'<noise type="gaussian">\n'
        f'{pad}    <mean>0.0</mean>\n'
        f'{pad}    <stddev>{n[f"{kind}_stddev_{unit}"]:.6g}</stddev>\n'
        f'{pad}    <bias_mean>0.0</bias_mean>\n'
        f'{pad}    <bias_stddev>{n[f"{kind}_bias_stddev_{unit}"]:.6g}</bias_stddev>\n'
        f'{pad}    <dynamic_bias_stddev>'
        f'{n[f"{kind}_dynamic_bias_stddev_{unit}"]:.6g}</dynamic_bias_stddev>\n'
        f'{pad}    <dynamic_bias_correlation_time>'
        f'{n[f"{kind}_bias_correlation_time_s"]:.6g}</dynamic_bias_correlation_time>\n'
        f'{pad}  </noise>'
    )
    return "\n".join(f"{pad}<{ax}>\n{pad}  {body}\n{pad}</{ax}>" for ax in ("x", "y", "z"))


# 一度纬度对应的地面距离，m/deg。用来绕过下面那个 Gazebo 的量纲 bug。
# 取 111320（WGS84 上随纬度在 110574–111694 之间变化，对噪声模型足够）。
_M_PER_DEG_LAT = 111320.0


def _navsat_noise(n: dict, kind: str, unit: str, indent: int) -> str:
    """NavSat 的水平/垂直两档噪声。

    水平与垂直分开是有物理依据的，不是凑数：卫星星座全在头顶半球，
    垂直方向的几何构型天然比水平差，所以垂直精度约为水平的 2 倍。

    ⚠️⚠️ **Gazebo 的量纲 bug：水平位置噪声实际按「度」施加，不是米。**

    SDF 规范（navsat.sdf）白纸黑字写着 "Noise parameters for horizontal
    position measurement, **in units of meters**"，但 gz-sensors 把它直接加在
    `math::Angle` 表示的经纬度上，而那个量的单位是度。

    2026-08-07 实测（车静止，真值纬度 31.2304）：填 stddev=2.0 时，
    量到的纬度在 31.67 / 35.15 之间跳 —— 散布 ≈ 2.0 **度** ≈ 217 km。
    垂直位置（altitude，普通 double）和测速（m/s）**都是对的**，
    只有水平位置这一项错。所以这里只换算这一项。

    换算后的实际噪声（本项目纬度 31.23°）：
        北向 σ = 标称值（1 度纬度 ≈ 111320 m，正是我们除掉的那个数）
        东向 σ = 标称值 × cos(31.23°) ≈ 0.855 × 标称
    15% 的各向异性，对一个 σ=2 m 的单点定位模型无关紧要（真实 GNSS 误差
    本来就是各向异性的），但要知道它在那里。

    ⚠️ **这个 workaround 有个反向风险**：Gazebo 哪天把 bug 修成米，
       这里就会把噪声缩小 111320 倍 —— GNSS 悄悄变成完美真值，
       而 P4 的判据会全部变绿。用 scripts/check_sensor_noise.py 定期实测拦它。
    """
    pad = " " * indent
    out = []
    for axis in ("horizontal", "vertical"):
        stddev = n[f"{kind}_{axis}_stddev_{unit}"]
        if kind == "position" and axis == "horizontal":
            stddev /= _M_PER_DEG_LAT
        out.append(f'{pad}<{axis}>\n'
                   f'{pad}  <noise type="gaussian">\n'
                   f'{pad}    <mean>0.0</mean>\n'
                   f'{pad}    <stddev>{stddev:.6g}</stddev>\n'
                   f'{pad}  </noise>\n'
                   f'{pad}</{axis}>')
    return "\n".join(out)


def render_sdf(p: dict) -> str:
    lim = p["limits"]
    sen = p["sensors"]
    d = derive(p)

    wheelbase = d["wheelbase"]
    track = d["track"]
    length = d["length"]
    com_h = d["com_h"]
    wheel_r = d["wheel_r"]
    wheel_w = d["wheel_w"]
    wheel_m = d["wheel_m"]
    chassis_mass = d["chassis_mass"]
    chassis_len_z = d["chassis_len_z"]
    chassis_center_x = d["chassis_center_x"]
    chassis_center_z = d["chassis_center_z"]
    chassis_width = d["chassis_width"]
    c_ixx, c_iyy, c_izz = d["c_ixx"], d["c_iyy"], d["c_izz"]
    w_ixx, w_iyy, w_izz = d["w_ixx"], d["w_iyy"], d["w_izz"]
    half_track = d["half_track"]
    steer_limit = d["steer_limit"]
    steer_p_gain = d["steer_p_gain"]

    lidar = sen["lidar"]
    imu = sen["imu"]
    gnss = sen["gnss"]

    def wheel_link(name: str, x: float, y: float) -> str:
        return f"""
    <link name="{name}">
      <pose>{x:.4f} {y:.4f} {wheel_r:.4f} 0 0 0</pose>
      <inertial>
        <mass>{wheel_m:.4f}</mass>
        <inertia>
          <ixx>{w_ixx:.6f}</ixx><iyy>{w_iyy:.6f}</iyy><izz>{w_izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 0 1.5707963 0 0</pose>
        <geometry>
          <cylinder><radius>{wheel_r:.4f}</radius><length>{wheel_w:.4f}</length></cylinder>
        </geometry>
        <surface>
          <friction>
            <ode><mu>1.2</mu><mu2>1.0</mu2></ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <pose>0 0 0 1.5707963 0 0</pose>
        <geometry>
          <cylinder><radius>{wheel_r:.4f}</radius><length>{wheel_w:.4f}</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
    </link>"""

    def steer_link(name: str, x: float, y: float) -> str:
        # 转向节。物理上它只是个转轴，但 SDF 里必须是个有质量的 link，
        # 质量给 0 会让求解器出 NaN。给个小值即可。
        return f"""
    <link name="{name}">
      <pose>{x:.4f} {y:.4f} {wheel_r:.4f} 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
    </link>"""

    return f"""<?xml version="1.0" ?>
<!-- ===========================================================================
     自动生成的文件 —— 不要手动编辑

     生成自：config/vehicle_params.yaml
     生成器：scripts/gen_vehicle_model.py
     手改会在下次运行生成器时被覆盖，而且会让本文件与 CARLA 侧的参数脱节
     （SPEC §4.1 车辆参数单一来源）。要改参数请改 YAML 再重新生成。

     坐标原点：base_link 位于**后轴中心、地面高度**（Autoware 惯例）
     =========================================================================== -->
<sdf version="1.9">
  <model name="{p['name']}">
    <link name="base_link">
      <inertial>
        <!-- 质心不在几何中心：真车质量集中在下半部，用 com_height_m 显式指定。
             放在几何中心会让车过弯时侧倾偏大，看着像悬架太软。 -->
        <pose>{chassis_center_x:.4f} 0 {com_h:.4f} 0 0 0</pose>
        <mass>{chassis_mass:.4f}</mass>
        <inertia>
          <ixx>{c_ixx:.6f}</ixx><iyy>{c_iyy:.6f}</iyy><izz>{c_izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>{chassis_center_x:.4f} 0 {chassis_center_z:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{length:.4f} {chassis_width:.4f} {chassis_len_z:.4f}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>{chassis_center_x:.4f} 0 {chassis_center_z:.4f} 0 0 0</pose>
        <geometry>
          <box><size>{length:.4f} {chassis_width:.4f} {chassis_len_z:.4f}</size></box>
        </geometry>
        <material>
          <ambient>0.2 0.35 0.6 1</ambient>
          <diffuse>0.25 0.45 0.8 1</diffuse>
        </material>
      </visual>

      <!-- ===================================================================
           传感器

           三个传感器都直接挂在 base_link 上、用 <pose> 表示外参，而不是各建
           一个独立的 link + fixed joint。理由：独立 link 需要质量和惯量，
           给 0 会让物理求解器出 NaN，给非 0 又凭空改变了整车质量分布 ——
           为了一个纯几何的安装点去污染动力学，不划算。

           <gz_frame_id> 让 Gazebo 在发布消息时用我们指定的 frame_id，
           而不是默认的作用域名（形如 ego_vehicle/base_link/lidar）。
           这一步是 TF 能对上的前提：URDF 里定义的 lidar_link 必须和这里
           报出来的 frame_id 同名，否则 RViz 会说 "no transform"。
           =================================================================== -->

      <sensor name="lidar" type="gpu_lidar">
        <pose>{lidar['mount_x_m']:.4f} {lidar['mount_y_m']:.4f} {lidar['mount_z_m']:.4f} 0 0 0</pose>
        <topic>/model/{p['name']}/lidar</topic>
        <gz_frame_id>lidar_link</gz_frame_id>
        <update_rate>{lidar['update_rate_hz']:.1f}</update_rate>
        <always_on>1</always_on>
        <!-- visualize 只影响 Gazebo GUI 里画不画射线。开着很直观，但几万条
             射线的绘制开销不小，会拉低 RTF —— 调试时临时开，平时关。 -->
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>{lidar['horizontal_samples']}</samples>
              <resolution>1</resolution>
              <min_angle>-3.141593</min_angle>
              <max_angle>3.141593</max_angle>
            </horizontal>
            <vertical>
              <samples>{lidar['channels']}</samples>
              <resolution>1</resolution>
              <min_angle>{lidar['vertical_fov_min_rad']:.6f}</min_angle>
              <max_angle>{lidar['vertical_fov_max_rad']:.6f}</max_angle>
            </vertical>
          </scan>
          <range>
            <min>{lidar['range_min_m']:.4f}</min>
            <max>{lidar['range_max_m']:.4f}</max>
            <!-- 距离量化步长。真实雷达受 ADC 位数限制，这里给 1 cm。 -->
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>{lidar['noise_stddev_m']:.4f}</stddev>
          </noise>
        </lidar>
      </sensor>

      <!-- ⚠️ IMU 与 NavSat 的 <noise> 用 **type 属性**（`<noise type="gaussian">`），
           而上面雷达的 <noise> 用 **<type> 子元素**。这两处写法**必须不同** ——
           不是笔误：lidar.sdf 自带一份 noise 定义（type 是子元素），
           而 imu.sdf / navsat.sdf 用的是共享的 noise.sdf（type 是属性）。
           搞反的后果两个方向不对称：IMU 用子元素写法是**硬解析错误**（会明确报错），
           雷达用属性写法只是警告并被静默归一化。已实测确认，见 CLAUDE.md 陷阱表。 -->
      <sensor name="imu" type="imu">
        <!-- 位姿为全零：IMU 就装在 base_link 原点，见 YAML 里关于杠杆臂的说明。 -->
        <pose>{imu['mount_x_m']:.4f} {imu['mount_y_m']:.4f} {imu['mount_z_m']:.4f} 0 0 0</pose>
        <topic>/model/{p['name']}/imu</topic>
        <gz_frame_id>base_link</gz_frame_id>
        <update_rate>{imu['update_rate_hz']:.1f}</update_rate>
        <always_on>1</always_on>
        <imu>
          <angular_velocity>
{_imu_axis_noise(imu['noise'], 'gyro', 'rad_s', 10)}
          </angular_velocity>
          <linear_acceleration>
{_imu_axis_noise(imu['noise'], 'accel', 'mps2', 10)}
          </linear_acceleration>
        </imu>
      </sensor>

      <sensor name="navsat" type="navsat">
        <pose>{gnss['mount_x_m']:.4f} {gnss['mount_y_m']:.4f} {gnss['mount_z_m']:.4f} 0 0 0</pose>
        <topic>/model/{p['name']}/navsat</topic>
        <gz_frame_id>gnss_link</gz_frame_id>
        <update_rate>{gnss['update_rate_hz']:.1f}</update_rate>
        <always_on>1</always_on>
        <navsat>
          <position_sensing>
{_navsat_noise(gnss['noise'], 'position', 'm', 12)}
          </position_sensing>
          <velocity_sensing>
{_navsat_noise(gnss['noise'], 'velocity', 'mps', 12)}
          </velocity_sensing>
        </navsat>
      </sensor>
    </link>
{steer_link('front_left_steer_link', wheelbase, half_track)}
{steer_link('front_right_steer_link', wheelbase, -half_track)}
{wheel_link('front_left_wheel', wheelbase, half_track)}
{wheel_link('front_right_wheel', wheelbase, -half_track)}
{wheel_link('rear_left_wheel', 0.0, half_track)}
{wheel_link('rear_right_wheel', 0.0, -half_track)}

    <!-- 转向关节：绕 Z 轴，角度限位取自 YAML 的 max_steer_angle_rad。

         <effort> 为什么必须写，且必须写这么大：
         AckermannSteering 插件对转向关节用的是**速度控制**。dartsim 在速度控制下
         只有当关节存在力矩上限时才会去检查位置限位 —— 不写 effort 的话，
         Gazebo 会直接报错说 "Velocity control does not respect positional limits"，
         而车的表现是**前轮可以转过 34.4° 的机械极限**，转弯半径比真车小得多。

         1e6 N·m 不是物理意义上的转向力矩（真车带助力也就几百 N·m 量级），
         它是求解器为了让限位生效所需要的「足够大」。真实的执行器力矩约束
         属于 P2 控制阶段的建模内容，那时会换成有物理依据的值。 -->
    <joint name="front_left_steer_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_left_steer_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>{-steer_limit:.4f}</lower><upper>{steer_limit:.4f}</upper>
          <effort>1.0e6</effort>
        </limit>
      </axis>
    </joint>
    <joint name="front_right_steer_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_right_steer_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>{-steer_limit:.4f}</lower><upper>{steer_limit:.4f}</upper>
          <effort>1.0e6</effort>
        </limit>
      </axis>
    </joint>

    <!-- 前轮滚动关节挂在转向节下，不是挂在车体上 —— 这样转向和滚动才是两个自由度 -->
    <joint name="front_left_wheel_joint" type="revolute">
      <parent>front_left_steer_link</parent>
      <child>front_left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>
    <joint name="front_right_wheel_joint" type="revolute">
      <parent>front_right_steer_link</parent>
      <child>front_right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <!-- 后轮为驱动轮 -->
    <joint name="rear_left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>rear_left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>
    <joint name="rear_right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>rear_right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <!-- =====================================================================
         Ackermann 转向系统

         订阅 Twist（linear.x = 车速 m/s，angular.z = 横摆角速度 rad/s），
         内部按 Ackermann 几何算出左右前轮各自的转角 —— 内外轮转角不同，
         这正是 Ackermann 相对于「两轮同角」的意义所在。

         S4 的 /vehicle_cmd（转角 + 加速度）到这里的转换由 gazebo_bridge 负责，
         不在本模型里做。
         ===================================================================== -->
    <plugin filename="gz-sim-ackermann-steering-system"
            name="gz::sim::systems::AckermannSteering">
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <left_steering_joint>front_left_steer_joint</left_steering_joint>
      <right_steering_joint>front_right_steer_joint</right_steering_joint>
      <kingpin_width>{track:.4f}</kingpin_width>
      <steering_limit>{steer_limit:.4f}</steering_limit>
      <!-- 转向伺服的 P 增益 = 1 / actuator.steer_response_time_s（推导量）。

           ⚠️ **这一行以前不在，用的是插件默认值 1.0，代价很具体**：
           实测转角阶跃的 63% 上升时间 1.198 s，比 Stanley 自己的闭环时间常数
           （1/k_e = 1.0 s）还大 —— 被控对象比控制器慢，弯道横向误差因此在
           ±0.8 m 之间震荡（同一条弯在 L1 的运动学模型上只有 1.6 cm）。
           CP-P2-B 就是被这一条卡住的。

           关键在于它**稳态是准的**（达成率 99.7–100.3%），只是慢：
           定转角测转弯半径的检查完全看不出问题，只有**阶跃响应**才量得到。
           见 docs/modules/control.md §3.9 与 scripts/probe_steering_response.py。 -->
      <steer_p_gain>{steer_p_gain:.4f}</steer_p_gain>
      <wheel_base>{wheelbase:.4f}</wheel_base>
      <wheel_separation>{track:.4f}</wheel_separation>
      <wheel_radius>{wheel_r:.4f}</wheel_radius>
      <min_velocity>{-lim['max_speed_mps']:.4f}</min_velocity>
      <max_velocity>{lim['max_speed_mps']:.4f}</max_velocity>
      <min_acceleration>{-lim['max_decel_mps2']:.4f}</min_acceleration>
      <max_acceleration>{lim['max_accel_mps2']:.4f}</max_acceleration>
      <topic>/model/{p['name']}/cmd_vel</topic>
      <odom_topic>/model/{p['name']}/odometry</odom_topic>
      <tf_topic>/model/{p['name']}/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>

    <!-- 关节状态发布。robot_state_publisher 拿它算 TF，RViz 里车轮才会转。

         显式指定 <topic>：默认话题名是 /world/<世界名>/model/<模型名>/joint_state，
         带世界名意味着换个世界跑就要改桥接配置。挂到模型作用域下就与世界解耦了。 -->
    <!-- 真值位姿发布器 —— SPEC §4.1 的 /ego_pose_gt 的来源（P4 新增）。

         ⚠️ **仅供评测打分与实测脚本使用，算法节点一律禁止订阅。**
            P4 的全部判据（定位横向误差、航向误差、绕环闭合误差）都要拿它当基准，
            所以它必须存在；但只要有一个算法节点偷偷订阅了它，
            整个定位模块的验收就变成了自己跟自己比 —— 而它看起来仍然是绿的。
            这与 reference_samples.csv 那条警告是同一类风险。

         发布的是**模型相对世界**的位姿。而 Gazebo 世界系就是本项目的 map 系
         （map→odom 由自车 spawn 位姿推出、odom→base_link 由 Ackermann 给），
         所以这个量直接就是真值的 map→base_link。

         发布频率取 50 Hz：比控制回路（20 Hz）高一档，
         保证评测时任意一拍都能找到时间上足够近的真值，又不至于刷屏。

         用 OdometryPublisher 而不是 PosePublisher，有两个具体理由：
           1. **frame 名可以指定**。PosePublisher 把 header.frame_id 填成父实体名，
              对模型来说就是**世界名**（实测得到 "campus_loop"）—— 换个世界这个
              字符串就变了，而下游拿它做 TF 查询会失败。这里能直接写死 map/base_link。
           2. 顺带给出**真值速度**（twist）。评测时"位置对了但速度估歪了"是一种
              真实的失效，有真值速度才量得出来。

         ⚠️ 它与车上那个 AckermannSteering 的 /odom **完全不是一回事**：
            后者是**轮速推算**（会漂移，正是 P4 要修正的对象），
            这个直接读物理引擎里的真实位姿。两者都叫 odometry，别搞混。

         ⚠️ <tf_topic> 指向一个**没人订阅**的话题：这个插件默认会发
            odom_frame → robot_base_frame 的 TF，而 map → base_link 若被它直接
            发出来，TF 树上 base_link 就有了两个父节点（另一个是 odom），
            tf2 会直接报错。真值只走话题，不进 TF。 -->
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>map</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/model/{p['name']}/pose_gt</odom_topic>
      <tf_topic>/model/{p['name']}/pose_gt_tf_unused</tf_topic>
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>/model/{p['name']}/joint_state</topic>
    </plugin>
  </model>
</sdf>
"""


def render_urdf(p: dict) -> str:
    """生成给 robot_state_publisher / RViz 用的 URDF。

    与 SDF 的分工：SDF 决定车怎么动，URDF 决定 TF 树长什么样、RViz 画什么。
    两者的关节名必须逐字相同 —— Gazebo 的 JointStatePublisher 按 SDF 里的
    关节名发 /joint_states，robot_state_publisher 按 URDF 里的关节名去查表。
    名字对不上不会报错，只是车轮在 RViz 里永远不转，很难联想到是名字问题。
    这也是两份都由本脚本生成的另一个理由。
    """
    lim = p["limits"]
    sen = p["sensors"]
    d = derive(p)

    wheel_r = d["wheel_r"]
    half_track = d["half_track"]
    steer_limit = d["steer_limit"]
    steer_p_gain = d["steer_p_gain"]

    # URDF 的 <limit> 要求填 effort 和 velocity。effort 对 RViz 显示没有影响
    # （robot_state_publisher 只做运动学），但字段是必填的，给个合理量级即可。
    steer_vel = lim["max_steer_rate_rad_s"]
    wheel_vel = lim["max_speed_mps"] / wheel_r   # 车速上限对应的车轮角速度 rad/s

    def wheel_link(name: str) -> str:
        return f"""
  <link name="{name}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{d['wheel_m']:.4f}"/>
      <inertia ixx="{d['w_ixx']:.6f}" iyy="{d['w_iyy']:.6f}" izz="{d['w_izz']:.6f}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <!-- URDF 的 cylinder 默认沿 Z 轴，车轮要绕 Y 轴滚，所以绕 X 转 90° -->
      <origin xyz="0 0 0" rpy="1.5707963 0 0"/>
      <geometry>
        <cylinder radius="{wheel_r:.4f}" length="{d['wheel_w']:.4f}"/>
      </geometry>
      <material name="tyre_black"/>
    </visual>
  </link>"""

    def steer_link(name: str) -> str:
        return f"""
  <link name="{name}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>"""

    def sensor_link(name: str, mount: dict, size: tuple[float, float, float]) -> str:
        sx, sy, sz = size
        return f"""
  <link name="{name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="{sx} {sy} {sz}"/></geometry>
      <material name="sensor_orange"/>
    </visual>
  </link>
  <joint name="{name}_joint" type="fixed">
    <parent link="base_link"/>
    <child link="{name}"/>
    <origin xyz="{mount['mount_x_m']:.4f} {mount['mount_y_m']:.4f} {mount['mount_z_m']:.4f}"
            rpy="0 0 0"/>
  </joint>"""

    return f"""<?xml version="1.0" ?>
<!-- ===========================================================================
     自动生成的文件 —— 不要手动编辑

     生成自：config/vehicle_params.yaml
     生成器：scripts/gen_vehicle_model.py

     用途：robot_state_publisher 读它 → 发布整棵 base_link 以下的 TF 子树，
           RViz 的 RobotModel 显示项读它 → 画出车。

     这里**没有**物理属性上的讲究（摩擦、驱动、插件都不在 URDF 里），
     那些在 models/ego_vehicle/model.sdf。惯量字段照抄只是为了满足
     URDF 解析器和 KDL 的要求，不参与任何仿真。

     坐标原点：base_link 位于**后轴中心、地面高度**（Autoware 惯例）
     =========================================================================== -->
<robot name="{p['name']}">

  <material name="chassis_blue"><color rgba="0.25 0.45 0.8 1.0"/></material>
  <material name="tyre_black"><color rgba="0.15 0.15 0.15 1.0"/></material>
  <material name="sensor_orange"><color rgba="0.95 0.55 0.10 1.0"/></material>

  <link name="base_link">
    <inertial>
      <origin xyz="{d['chassis_center_x']:.4f} 0 {d['com_h']:.4f}" rpy="0 0 0"/>
      <mass value="{d['chassis_mass']:.4f}"/>
      <inertia ixx="{d['c_ixx']:.6f}" iyy="{d['c_iyy']:.6f}" izz="{d['c_izz']:.6f}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="{d['chassis_center_x']:.4f} 0 {d['chassis_center_z']:.4f}" rpy="0 0 0"/>
      <geometry>
        <box size="{d['length']:.4f} {d['chassis_width']:.4f} {d['chassis_len_z']:.4f}"/>
      </geometry>
      <material name="chassis_blue"/>
    </visual>
  </link>
{steer_link('front_left_steer_link')}
{steer_link('front_right_steer_link')}
{wheel_link('front_left_wheel')}
{wheel_link('front_right_wheel')}
{wheel_link('rear_left_wheel')}
{wheel_link('rear_right_wheel')}

  <!-- 转向关节：把转向节挂到车体上，绕 Z 转 -->
  <joint name="front_left_steer_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_steer_link"/>
    <origin xyz="{d['wheelbase']:.4f} {half_track:.4f} {wheel_r:.4f}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{-steer_limit:.4f}" upper="{steer_limit:.4f}"
           effort="1000.0" velocity="{steer_vel:.4f}"/>
  </joint>
  <joint name="front_right_steer_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_right_steer_link"/>
    <origin xyz="{d['wheelbase']:.4f} {-half_track:.4f} {wheel_r:.4f}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{-steer_limit:.4f}" upper="{steer_limit:.4f}"
           effort="1000.0" velocity="{steer_vel:.4f}"/>
  </joint>

  <!-- 前轮滚动关节挂在转向节下（不是车体），转向与滚动才是两个独立自由度。
       type=continuous：车轮可以无限圈转，不设上下限。 -->
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="front_left_steer_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="{wheel_vel:.4f}"/>
  </joint>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="front_right_steer_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="{wheel_vel:.4f}"/>
  </joint>

  <!-- 后轮为驱动轮，直接挂在车体上 -->
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="0 {half_track:.4f} {wheel_r:.4f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="{wheel_vel:.4f}"/>
  </joint>
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="0 {-half_track:.4f} {wheel_r:.4f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="{wheel_vel:.4f}"/>
  </joint>

  <!-- ======================================================================
       传感器坐标系

       这几个 fixed joint 就是**外参**。gazebo_bridge 把点云从 lidar_link
       变换到 base_link，查的正是这里发布的 TF。所以「URDF 里的安装位置」
       和「SDF 里 <sensor> 的 <pose>」必须一致 —— 两者都来自 YAML 的
       sensors 段，这就是它们不会漂移的原因。

       注意没有 imu_link：IMU 装在 base_link 原点，两者是同一个坐标系
       （见 vehicle_params.yaml 里关于杠杆臂的说明）。
       ====================================================================== -->
{sensor_link('lidar_link', sen['lidar'], (0.10, 0.10, 0.08))}
{sensor_link('gnss_link', sen['gnss'], (0.08, 0.08, 0.03))}

</robot>
"""


# 输出物清单。--check 会逐个比对，任何一个与 YAML 不同步都算失败。
OUTPUTS = [
    (SDF_FILE, render_sdf),
    (URDF_FILE, render_urdf),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="不写文件，只校验生成物是否与 YAML 同步；不同步则退出码 1")
    args = ap.parse_args()

    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))

    if args.check:
        stale = 0
        for path, fn in OUTPUTS:
            rel = path.relative_to(REPO_ROOT)
            if not path.exists():
                print(f"✗ {rel} 不存在，请运行 scripts/gen_vehicle_model.py 生成",
                      file=sys.stderr)
                stale = 1
            elif path.read_text(encoding="utf-8") != fn(params):
                print(f"✗ {rel} 与 {PARAMS_FILE.relative_to(REPO_ROOT)} 不同步。\n"
                      f"  要么有人手改了生成物，要么改了 YAML 忘了重新生成。\n"
                      f"  运行：python3 scripts/gen_vehicle_model.py", file=sys.stderr)
                stale = 1
            else:
                print(f"✓ {rel} 与参数文件同步")
        return stale

    for path, fn in OUTPUTS:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(fn(params), encoding="utf-8")
        print(f"已生成 {path.relative_to(REPO_ROOT)}")

    geo, lidar = params["geometry"], params["sensors"]["lidar"]
    print(f"  轴距 {geo['wheelbase_m']} m  "
          f"轮距 {geo['track_width_m']} m  "
          f"整备质量 {params['mass']['total_kg']} kg  "
          f"最大转角 {params['limits']['max_steer_angle_rad']} rad")
    print(f"  激光雷达 {lidar['channels']} 线 × {lidar['horizontal_samples']} 点 "
          f"@ {lidar['update_rate_hz']} Hz  量程 {lidar['range_max_m']} m  "
          f"→ {lidar['channels'] * lidar['horizontal_samples']:,} 点/帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
