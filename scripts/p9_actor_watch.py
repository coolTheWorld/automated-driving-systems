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

"""P9-S2 黄金线索的**独立**目击者：第二个客户端定期普查 get_actors.

sidecar 自己也普查（_npc_census），但它是嫌疑人之一 —— 「sidecar 传感器
回调线程与 client 的竞态把句柄炸了」这条嫌疑要用一个与 sidecar 无关的
进程来排除/坐实。本脚本**只读**：不 apply_settings、不 load_world、不 spawn、
不 destroy —— 任何写操作都会让它自己成为新的嫌疑人。

用法（云机容器内，栈起之前或之后都行）：
    python3 scripts/p9_actor_watch.py --period-s 10 --out /tmp/actor_watch.log

输出每行：墙钟 | 仿真钟 | actor 总数 | 每个 vehicle/walker 的 id type role (x,y,z)
actor 消失时打一行 **VANISHED**（含最后一次见到它的 z 与时刻）。
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    """入口：只读普查循环."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--period-s', type=float, default=10.0)
    parser.add_argument('--out', default='-', help='日志文件；- 为 stdout')
    parser.add_argument('--duration-s', type=float, default=600.0)
    args = parser.parse_args()

    try:
        import carla  # noqa: WPS433 —— 惰性：本机没 wheel 时给出指名错误
    except ImportError:
        print('carla PythonAPI 不可用：本脚本只在云机上跑', file=sys.stderr)
        return 2

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()   # 只读句柄；不碰 settings
    out = sys.stdout if args.out == '-' else open(args.out, 'a', encoding='utf-8')

    seen: dict[int, tuple[float, float, str]] = {}   # id → (最后仿真钟, 最后 z, 描述)
    t_end = time.monotonic() + args.duration_s
    while time.monotonic() < t_end:
        try:
            snapshot = world.get_snapshot()
            actors = world.get_actors()
        except Exception as error:  # noqa: B902 —— 服务器崩了也要有最后一行
            print(f'{time.strftime("%H:%M:%S")} | 普查失败：{error!r}', file=out, flush=True)
            time.sleep(args.period_s)
            continue
        sim_t = snapshot.timestamp.elapsed_seconds
        movers = [a for a in actors if a.type_id.startswith(('vehicle.', 'walker.'))]
        now_ids = set()
        parts = []
        for actor in movers:
            loc = actor.get_location()
            role = actor.attributes.get('role_name', '')
            desc = f'{actor.id}:{actor.type_id.split(".", 1)[1]}[{role}]'
            parts.append(f'{desc}({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})')
            seen[actor.id] = (sim_t, loc.z, desc)
            now_ids.add(actor.id)
        vanished = [i for i in seen if i not in now_ids]
        for actor_id in vanished:
            last_t, last_z, desc = seen.pop(actor_id)
            print(f'{time.strftime("%H:%M:%S")} | sim {sim_t:7.1f} | **VANISHED** {desc} '
                  f'最后见于 sim {last_t:.1f} z={last_z:.1f}', file=out, flush=True)
        print(f'{time.strftime("%H:%M:%S")} | sim {sim_t:7.1f} | 总 {len(actors)} | '
              + ' '.join(parts), file=out, flush=True)
        time.sleep(args.period_s)
    return 0


if __name__ == '__main__':
    sys.exit(main())
