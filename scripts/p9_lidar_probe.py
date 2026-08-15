#!/usr/bin/env python3
# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""P9 传感器裸测：sidecar 同款雷达在同一张生成地图上，逐 tick 看它到底扫到了什么.

窗口 4 第一轮（2026-08-15）实测把问题钉到了**传感器本身**：processed 帧里
一帧只有墙有离地点 —— NPC 车 3.5–35 m 全程 0 点、行人 8 m 有 630 点而 15 m
以外 0 点；raw 帧方位角直方图**只有一半**（[0°,180°) 每通道 1800 点、
另一半几乎为空），且 63% 的点是 0.15–0.3 m 处的自反射。分割/聚类/跟踪
都在这之后 —— 先把雷达本身量清楚再谈感知参数。

本脚本**不依赖 ROS、不依赖 sidecar**（但复刻它的世界参数与雷达属性），
在空闲的 CARLA 服务端上：生成同一张 campus 地图 → 同步 0.05 → spawn 自车
（citroen.c3）+ 同款雷达 → 在指定位置放一辆静止 NPC 车和一个行人 →
逐 tick 打印：raw horizontal_angle、点数、方位角 8 扇区直方图、俯仰覆盖、
NPC 车/行人真值邻域点数（正窗与镜像窗都数，y 符号一并核）。

用法（云机容器内，**栈没在跑**时）：
    python3 scripts/p9_lidar_probe.py                       # 默认：雷达 z=1.6，转速 10 Hz
    python3 scripts/p9_lidar_probe.py --lidar-z 2.5         # 抬高：区分「车顶自反射」与别的
    python3 scripts/p9_lidar_probe.py --rotation-hz 20      # 每 tick 整圈：区分「半圈盲」的来源
    python3 scripts/p9_lidar_probe.py --npc-ahead 8 --npc-lateral 3.5
它会把世界留在异步模式、销毁自己 spawn 的一切。
"""

from __future__ import annotations

import argparse
import collections
import math
import sys
import time

import numpy as np
import yaml


def build_lidar_attrs(vehicle_yaml: str, rotation_hz: float, dropoff: bool) -> dict:
    """与 sidecar `_spawn_sensors` 同一份 vehicle_params 推出的雷达属性（复刻，非导入 ——
    sidecar 模块 import 时要 rclpy，本脚本要能在只有 carla wheel 的解释器里跑）."""
    with open(vehicle_yaml, encoding='utf-8') as f:
        vehicle = yaml.safe_load(f)
    lidar = vehicle['sensors']['lidar']
    attrs = {
        'channels': str(lidar['channels']),
        'rotation_frequency': str(rotation_hz),
        # 点率按 sidecar 同一公式：channels × horizontal_samples × 10（sidecar 写死 10）
        'points_per_second': str(lidar['channels'] * lidar['horizontal_samples'] * 10),
        'range': '30.0',
        'upper_fov': str(math.degrees(lidar['vertical_fov_max_rad'])),
        'lower_fov': str(math.degrees(lidar['vertical_fov_min_rad'])),
        'noise_stddev': '0.01',
    }
    if not dropoff:
        attrs.update({
            'dropoff_general_rate': '0.0',
            'dropoff_intensity_limit': '0.0',
            'dropoff_zero_intensity': '0.0',
            'atmosphere_attenuation_rate': '0.0',
        })
    mount = (lidar['mount_x_m'], lidar['mount_y_m'], lidar['mount_z_m'])
    geo = vehicle['geometry']
    axle_to_center = 0.5 * geo['length_m'] - geo['rear_overhang_m']
    return attrs, mount, axle_to_center


def to_sensor_frame(sensor_tf, world_loc):
    """世界点 → 传感器局部（UE 左手系：x 前 y 右，yaw 顺时针为正）."""
    rx = world_loc.x - sensor_tf.location.x
    ry = world_loc.y - sensor_tf.location.y
    yaw = math.radians(sensor_tf.rotation.yaw)
    return (math.cos(yaw) * rx + math.sin(yaw) * ry,
            -math.sin(yaw) * rx + math.cos(yaw) * ry,
            world_loc.z - sensor_tf.location.z)


def describe(points: np.ndarray, targets: dict) -> str:
    """一 tick 的点云画像：方位角 8 扇区、俯仰覆盖、近处自反射、真值邻域."""
    if points.shape[0] == 0:
        return '  （0 点）'
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    rng = np.hypot(x, y)
    az = np.degrees(np.arctan2(y, x))
    el = np.degrees(np.arctan2(z, rng))
    sectors = collections.Counter(int(a // 45) * 45 for a in az)
    sector_text = ' '.join(f'[{k:+4d}]{sectors.get(k, 0)}' for k in range(-180, 180, 45))
    lines = [f'  方位角 45° 扇区：{sector_text}',
             f'  俯仰 min/max {el.min():.1f}/{el.max():.1f}°，距离中位 {np.median(rng):.2f} m，'
             f'<1 m 自反射 {int((rng < 1.0).sum())} 点，>1 m {int((rng >= 1.0).sum())} 点']
    for name, (tx, ty, _) in targets.items():
        far = np.hypot(x - tx, y - ty) < 2.5
        mirror = np.hypot(x - tx, y + ty) < 2.5
        # 正窗里**离地 >0.3 m** 的点的质心相对真值的偏差（沿视线 / 横向）：
        # 判「感知看到的目标在不在真值说的地方」——窗口 4 行人真值系统性落后 1.1 m
        # 的案子就靠它在裸测里复现/排除
        body = far & (z > 0.3 - 1.6)   # 传感器系 z：地面在 −1.6 附近
        centroid = ''
        if body.sum() >= 5:
            cx, cy = float(x[body].mean()), float(y[body].mean())
            rng_t = math.hypot(tx, ty)
            ux, uy = tx / rng_t, ty / rng_t
            along = (cx - tx) * ux + (cy - ty) * uy      # 正 = 质心比真值更远
            cross = -(cx - tx) * uy + (cy - ty) * ux
            centroid = f'，离地点质心偏差 沿视线 {along:+.2f} m / 横向 {cross:+.2f} m（{int(body.sum())} 点）'
        lines.append(f'  {name} 真值(传感器系 x={tx:.1f} y={ty:.1f})：正窗 {int(far.sum())} 点 '
                     f'/ 镜像窗 {int(mirror.sum())} 点{centroid}')
    return '\n'.join(lines)


def main() -> int:
    """入口."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--xodr', default='/workspace/maps/campus.xodr')
    parser.add_argument('--vehicle-yaml', default='/workspace/config/vehicle_params.yaml')
    parser.add_argument('--lidar-z', type=float, default=None,
                        help='覆盖雷达挂高（默认取 vehicle_params 的 mount_z_m）')
    parser.add_argument('--rotation-hz', type=float, default=10.0)
    parser.add_argument('--dropoff', action='store_true', help='保留 CARLA 默认丢点模型')
    parser.add_argument('--npc-ahead', type=float, default=15.0, help='NPC 车在自车前方多少米')
    parser.add_argument('--npc-lateral', type=float, default=3.5,
                        help='NPC 车在自车左侧多少米（ROS 约定，正=左）')
    parser.add_argument('--walker-ahead', type=float, default=12.0)
    parser.add_argument('--walker-lateral', type=float, default=-3.0)
    parser.add_argument('--walker-move-mps', type=float, default=0.0,
                        help='>0 时每 tick 用 set_transform 把行人沿 +x 瞬移 v·0.05 m（复刻 sidecar 驱动法）')
    parser.add_argument('--walker-z', type=float, default=None,
                        help='瞬移时的 root z（sidecar 旧值 0.2 = 埋地；默认 = 落定高度）')
    parser.add_argument('--ticks', type=int, default=6)
    parser.add_argument('--no-generate', action='store_true',
                        help='不重生成地图（世界已是 campus 时省 30 s）')
    parser.add_argument('--additional-width', type=float, default=6.0,
                        help='生成世界的路肩宽（sidecar 现值 6.0）；试「墙退出雷达量程」用')
    parser.add_argument('--wall-height', type=float, default=0.0,
                        help='生成世界的护栏墙高（sidecar 现值 0.0 —— 窗口 4 起撤墙）')
    parser.add_argument('--ego-x', type=float, default=30.0, help='自车 ROS x')
    parser.add_argument('--ego-y', type=float, default=-51.75, help='自车 ROS y')
    parser.add_argument('--ego-yaw-deg', type=float, default=0.0, help='自车 ROS 朝向（度）')
    parser.add_argument('--no-targets', action='store_true', help='不放 NPC 车/行人（只看场景）')
    parser.add_argument('--dump', default=None,
                        help='把最后两拍（整圈）以 base_link 系 x,y,z 落盘（ROS 约定：y 已翻、z 已加挂高）')
    args = parser.parse_args()

    try:
        import carla
    except ImportError:
        print('carla PythonAPI 不可用：本脚本只在云机上跑', file=sys.stderr)
        return 2

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.get_world()
    settings = world.get_settings()
    if settings.synchronous_mode:   # 上任死得不干净：先解锁（同 sidecar 开锁器）
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    if not args.no_generate:
        with open(args.xodr, encoding='utf-8') as f:
            xodr = f.read()
        params = carla.OpendriveGenerationParameters(
            vertex_distance=0.5, max_road_length=50.0, wall_height=args.wall_height,
            additional_width=args.additional_width, smooth_junctions=False,
            enable_mesh_visibility=True)
        world = client.generate_opendrive_world(xodr, params)   # 与 sidecar 同参数
    else:
        # 复用现成世界时先清场：sidecar 退出**不销毁**它的 actor（下一次 generate
        # 才会清掉），留在 spawn 点上的自车会让这里的 spawn 报 collision。
        time.sleep(1.0)   # 新客户端第一份 episode state 要等一拍（见 p9_actor_watch）
        leftovers = [a for a in world.get_actors()
                     if a.type_id.startswith(('vehicle.', 'walker.', 'sensor.', 'static.prop'))]
        for actor in leftovers:
            try:
                actor.destroy()
            except Exception:  # noqa: B902
                pass
        if leftovers:
            print(f'清掉 {len(leftovers)} 个遗留 actor')
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    attrs, mount, axle_to_center = build_lidar_attrs(
        args.vehicle_yaml, args.rotation_hz, args.dropoff)
    lidar_z = args.lidar_z if args.lidar_z is not None else mount[2]
    library = world.get_blueprint_library()
    spawned = []
    try:
        # 自车：spawn 位姿同 sidecar（ROS (30, −51.75, yaw 0) → CARLA (30, 51.75)）
        ego_bp = library.find('vehicle.citroen.c3')
        ego_bp.set_attribute('role_name', 'ego_vehicle')
        ego = world.spawn_actor(ego_bp, carla.Transform(
            carla.Location(x=args.ego_x, y=-args.ego_y, z=0.3),
            carla.Rotation(yaw=-args.ego_yaw_deg)))
        spawned.append(ego)
        # 雷达：同款属性，同款安装偏移
        bp = library.find('sensor.lidar.ray_cast')
        for key, value in attrs.items():
            if bp.has_attribute(key):
                bp.set_attribute(key, value)
        lidar_tf = carla.Transform(carla.Location(
            x=mount[0] - axle_to_center, y=-mount[1], z=lidar_z))
        lidar = world.spawn_actor(bp, lidar_tf, attach_to=ego)
        spawned.append(lidar)
        # 静止目标：NPC 车（sidecar 同款蓝图）+ 行人，物理开、重力开，落在路面上
        npc = walker = None
        if not args.no_targets:
            npc_bp = library.find('vehicle.nissan.micra')
            npc = world.spawn_actor(npc_bp, carla.Transform(
                carla.Location(x=args.ego_x + args.npc_ahead, y=-args.ego_y - args.npc_lateral,
                               z=0.3),
                carla.Rotation(yaw=180.0)))
            spawned.append(npc)
            walker_bp = library.filter('walker.pedestrian.*')[0]
            walker = world.spawn_actor(walker_bp, carla.Transform(
                carla.Location(x=args.ego_x + args.walker_ahead,
                               y=-args.ego_y - args.walker_lateral, z=1.0)))
            spawned.append(walker)

        frames = []
        lidar.listen(lambda data: frames.append((
            data.frame, data.horizontal_angle,
            np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4).copy())))
        # 让物理落定（车从 z=0.3 落地、行人落地）再采样
        for _ in range(20):
            world.tick()
        frames.clear()
        print(f'雷达属性：{attrs}  安装 z={lidar_z:.2f}（车辆原点系）')
        for name, actor in (('自车', ego), ('npc_car', npc), ('walker', walker)):
            if actor is not None:
                bb = actor.bounding_box
                loc = actor.get_location()
                print(f'{name} bbox extent=({bb.extent.x:.3f},{bb.extent.y:.3f},{bb.extent.z:.3f}) '
                      f'center offset=({bb.location.x:.3f},{bb.location.y:.3f},'
                      f'{bb.location.z:.3f}) 落定后 actor z={loc.z:.3f}'
                      '（重力落地后原点离地高 = 真值 z 该填的数）')
        last_two = []
        walker_pose = None
        if walker is not None and args.walker_move_mps > 0.0:
            loc0 = walker.get_transform().location
            walker_pose = [loc0.x, loc0.y, args.walker_z if args.walker_z is not None else loc0.z]
        for tick in range(args.ticks):
            if walker_pose is not None:   # sidecar 同款：先算新位姿、set_transform、再 tick
                walker_pose[0] += args.walker_move_mps * 0.05
                walker.set_transform(carla.Transform(
                    carla.Location(x=walker_pose[0], y=walker_pose[1], z=walker_pose[2]),
                    carla.Rotation(yaw=0.0)))
            world.tick()
            time.sleep(0.05)   # 让回调把这一拍的数据送到
            if not frames:
                print(f'tick {tick}: 没收到雷达数据')
                continue
            frame_id, hangle, points = frames[-1]
            frames.clear()
            last_two = (last_two + [points])[-2:]
            sensor_tf = lidar.get_transform()
            targets = {}
            if npc is not None:
                targets['npc_car'] = to_sensor_frame(sensor_tf, npc.get_transform().location)
                targets['walker'] = to_sensor_frame(sensor_tf, walker.get_transform().location)
                if walker_pose is not None:
                    # 脚本位姿（sidecar 的 pose_gt 就是它）与 actor 实际位姿分开看
                    targets['walker@script'] = to_sensor_frame(
                        sensor_tf, carla.Location(
                            x=walker_pose[0], y=walker_pose[1], z=walker_pose[2]))
            print(f'tick {tick} frame {frame_id}: horizontal_angle={hangle:.4f}  '
                  f'点数 {points.shape[0]}')
            print(describe(points, targets))
        # 参考：真值到传感器的距离与方位（用来读上面的表）
        for name, actor in (('npc_car', npc), ('walker', walker)):
            if actor is None:
                continue
            tx, ty, tz = to_sensor_frame(lidar.get_transform(), actor.get_transform().location)
            print(f'{name}：传感器系 ({tx:.2f}, {ty:.2f}, {tz:.2f})  距离 {math.hypot(tx, ty):.1f} m '
                  f'方位 {math.degrees(math.atan2(ty, tx)):+.1f}°（UE 约定：正=右）')
        if args.dump and len(last_two) == 2:
            # 整圈 = 相邻两拍；转 ROS 约定：y 翻号、z 加挂高（base_link 在地面）
            whole = np.concatenate(last_two)[:, :3].copy()
            whole[:, 1] *= -1.0
            whole[:, 2] += lidar_z
            np.savetxt(args.dump, whole, fmt='%.3f', delimiter=',', header='x,y,z', comments='')
            print(f'整圈 {whole.shape[0]} 点已落盘 → {args.dump}（base_link 系）')
    finally:
        try:
            lidar.stop()
        except Exception:  # noqa: B902
            pass
        for actor in reversed(spawned):
            try:
                actor.destroy()
            except Exception:  # noqa: B902
                pass
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    return 0


if __name__ == '__main__':
    sys.exit(main())
