#!/usr/bin/env python3
"""从 config/vehicle_params.yaml 生成 models/ego_vehicle/model.sdf。

为什么要生成而不是手写 SDF
--------------------------
SPEC §4.1 把「车辆参数单一来源」列为强制要求：轴距、最大转角、质量等
必须只在 config/vehicle_params.yaml 里定义一次，Gazebo 与 CARLA 都从它读。

手写 SDF 做不到这一点。轴距这一个数字在 SDF 里就要出现四次以上
（前后轮位置、Ackermann 插件的 wheel_base、车体盒的中心偏移），
改一处漏三处的事迟早会发生。而它的后果不是报错，是**车在两个仿真里
开出不同的轨迹**，然后你去怀疑控制器 —— SPEC §4.1 说的「行为漂移」。

所以 SDF 是生成物。改参数改 YAML，然后重新跑本脚本。

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
OUTPUT_FILE = REPO_ROOT / "models" / "ego_vehicle" / "model.sdf"


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


def render(p: dict) -> str:
    geo = p["geometry"]
    whl = p["wheel"]
    lim = p["limits"]

    wheelbase = geo["wheelbase_m"]
    track = geo["track_width_m"]
    length = geo["length_m"]
    height = geo["height_m"]
    rear_overhang = geo["rear_overhang_m"]
    com_h = geo["com_height_m"]
    clearance = geo["ground_clearance_m"]

    wheel_r = whl["radius_m"]
    wheel_w = whl["width_m"]
    wheel_m = whl["mass_kg"]

    total_mass = p["mass"]["total_kg"]
    chassis_mass = total_mass - 4.0 * wheel_m

    # ---- 坐标原点的选择 ----
    # base_link 放在**后轴中心、地面高度**（Autoware 惯例）。
    # 自行车模型、Stanley、纯追踪都以后轴为参考点推导，原点选这里能让
    # 控制算法直接使用位姿，不必到处做偏移换算 —— 偏移换算写错是常见 bug。
    chassis_len_z = height - clearance
    chassis_center_x = length / 2.0 - rear_overhang   # 车体盒中心相对后轴的纵向偏移
    chassis_center_z = clearance + chassis_len_z / 2.0

    # 车体盒宽度取「轮距 - 轮宽」，正好让车轮内侧与车体侧面相切。
    # 直接用 width_m(1.8) 会把车轮整个包进车体里 —— 物理上无害（同一模型内
    # 各 link 默认不自碰撞），但看上去像块砖，调试转向时根本看不出前轮转了没有。
    # 整车外廓宽度 width_m 是**含轮胎的保守包络**，供规划做碰撞检查，不用于建模。
    chassis_width = track - wheel_w

    c_ixx, c_iyy, c_izz = box_inertia(chassis_mass, length, chassis_width, chassis_len_z)
    w_ixx, w_iyy, w_izz = wheel_inertia(wheel_m, wheel_r, wheel_w)

    half_track = track / 2.0
    steer_limit = lim["max_steer_angle_rad"]

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
    </link>
{steer_link('front_left_steer_link', wheelbase, half_track)}
{steer_link('front_right_steer_link', wheelbase, -half_track)}
{wheel_link('front_left_wheel', wheelbase, half_track)}
{wheel_link('front_right_wheel', wheelbase, -half_track)}
{wheel_link('rear_left_wheel', 0.0, half_track)}
{wheel_link('rear_right_wheel', 0.0, -half_track)}

    <!-- 转向关节：绕 Z 轴，限位取自 YAML 的 max_steer_angle_rad -->
    <joint name="front_left_steer_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_left_steer_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{-steer_limit:.4f}</lower><upper>{steer_limit:.4f}</upper></limit>
      </axis>
    </joint>
    <joint name="front_right_steer_joint" type="revolute">
      <parent>base_link</parent>
      <child>front_right_steer_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{-steer_limit:.4f}</lower><upper>{steer_limit:.4f}</upper></limit>
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

    <!-- 关节状态发布，S3 建 TF 树时要用 -->
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
    </plugin>
  </model>
</sdf>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="不写文件，只校验生成物是否与 YAML 同步；不同步则退出码 1")
    args = ap.parse_args()

    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
    content = render(params)

    if args.check:
        if not OUTPUT_FILE.exists():
            print(f"✗ {OUTPUT_FILE.relative_to(REPO_ROOT)} 不存在，请运行 "
                  f"scripts/gen_vehicle_model.py 生成", file=sys.stderr)
            return 1
        if OUTPUT_FILE.read_text(encoding="utf-8") != content:
            print(f"✗ {OUTPUT_FILE.relative_to(REPO_ROOT)} 与 "
                  f"{PARAMS_FILE.relative_to(REPO_ROOT)} 不同步。\n"
                  f"  要么有人手改了生成物，要么改了 YAML 忘了重新生成。\n"
                  f"  运行：python3 scripts/gen_vehicle_model.py", file=sys.stderr)
            return 1
        print(f"✓ {OUTPUT_FILE.relative_to(REPO_ROOT)} 与参数文件同步")
        return 0

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_FILE.write_text(content, encoding="utf-8")
    print(f"已生成 {OUTPUT_FILE.relative_to(REPO_ROOT)}")
    print(f"  轴距 {params['geometry']['wheelbase_m']} m  "
          f"轮距 {params['geometry']['track_width_m']} m  "
          f"整备质量 {params['mass']['total_kg']} kg  "
          f"最大转角 {params['limits']['max_steer_angle_rad']} rad")
    return 0


if __name__ == "__main__":
    sys.exit(main())
