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

"""云机上「亲眼看」CARLA：给自车挂一台追尾相机，MJPEG 通过 HTTP 直播到本机浏览器.

服务端跑在 `-RenderOffScreen`（无头，SPEC §4.1 环境 B 的前提），没有窗口可看；
本脚本作为**第二个只读客户端**挂三台 RGB 相机 —— 追尾、自车正上方 35 m 的跟车俯视、
环线上空 150 m 的全图（找不到自车时只剩全图），把每帧编成 JPEG，用
multipart/x-mixed-replace 直播。

用法（云机容器内，栈起不起都行；栈起落之间它会自己重挂）：
    docker exec -d ads-dev python3 /workspace/scripts/carla_view.py          # 默认 :8080
本机：
    ssh -p <port> -L 8080:127.0.0.1:8080 root@<host>      # 保持这个 ssh 开着
    浏览器打开 http://localhost:8080                       # 追尾；/top 跟车俯视；/map 全图
然后照常派轮（l3c_*_round.sh），车一 spawn 画面就切到追尾视角。

⚠️ 只读：不 apply_settings、不重载世界、不 spawn 车 —— 相机是唯一的写操作，
   而 CARLA 的 spawn 碰撞检查对 sensor 不生效，也不占物理场景。
   同步模式下相机跟着 sidecar 的 tick 出帧（20 FPS），本脚本自己不 tick。
⚠️ 服务端渲染多一台 960×540 相机会吃 GPU：4090 上 RTF 判据不受影响（实测），
   看完记得停掉（docker exec ads-dev pkill -f carla_view）。
"""

from __future__ import annotations

import argparse
import io
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

# 追尾视角（相对自车原点 = 包围盒中心，CARLA 左手系：x 前 z 上）
CHASE_LOCATION = (-7.5, 0.0, 3.2)
CHASE_PITCH_DEG = -14.0
# 跟车俯视：挂在自车正上方 35 m 竖直向下（90° 视场 ⟹ 70 m 见方，车 ~50 像素、
# 行人几像素也看得见）—— 看让行/横穿/跟停的相对几何用这个
TOP_HEIGHT_M = 35.0
# 全图：整个环线正上方（环线 180×100 m 以原点为中心，150 m 高 + 90° 视场罩住 300 m
# 见方）。⚠️ 这个高度上一辆车只有 13×6 像素（0.31 m/px），JPEG 一压就是个白点 ——
# 它是看「谁在哪」的示意图，不是看车的；实测 150 m 时车没被剔除，只是小
MAP_LOCATION = (0.0, 0.0, 150.0)
MAP_PITCH_DEG = -89.9

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>CARLA</title>
<style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}
.bar{padding:6px 10px}.bar a{color:#8cf;margin-right:14px}img{display:block;max-width:100vw}</style>
</head><body><div class="bar"><a href="/">追尾</a><a href="/top">跟车俯视</a><a href="/map">全图</a>
<span id="s"></span></div><img id="v" src="/stream?view=VIEW"><script>
setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>{document.getElementById('s').textContent=t}),1000)
</script></body></html>""".encode('utf-8')


class Frames:
    """最新一帧 JPEG（每个视角一份）+ 状态文本，线程安全."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = {'chase': None, 'top': None, 'map': None}
        self.stamp = {'chase': 0.0, 'top': 0.0, 'map': 0.0}
        self.status = '连接中…'

    def put(self, view, data):
        with self.lock:
            self.jpeg[view] = data
            self.stamp[view] = time.monotonic()

    def get(self, view):
        with self.lock:
            return self.jpeg[view], self.stamp[view]


FRAMES = Frames()


def encode_jpeg(image, quality):
    """carla.Image（BGRA）→ JPEG bytes."""
    from PIL import Image  # 惰性：容器里有，别处不一定
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    rgb = array[:, :, :3][:, :, ::-1]      # BGRA → RGB（copy 由 tobytes 隐式完成）
    buffer = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).save(buffer, format='JPEG', quality=quality)
    return buffer.getvalue()


class Handler(BaseHTTPRequestHandler):
    """/ 与 /top 是页面，/stream?view= 是 MJPEG，/frame.jpg 单帧，/status 文本."""

    def log_message(self, *_):  # 安静
        pass

    def do_GET(self):  # noqa: N802
        path, _, query = self.path.partition('?')
        view = 'chase'
        for name in ('top', 'map'):
            if f'view={name}' in query or path == f'/{name}':
                view = name
        if path in ('/', '/top', '/map'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(PAGE.replace(b'VIEW', view.encode()))
        elif path == '/status':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.end_headers()
            self.wfile.write(FRAMES.status.encode())
        elif path == '/frame.jpg':
            data, _ = FRAMES.get(view)
            if data is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last = -1.0
            try:
                while True:
                    data, stamp = FRAMES.get(view)
                    if data is not None and stamp != last:
                        last = stamp
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n')
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_response(404)
            self.end_headers()


def find_ego(world):
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'ego_vehicle':
            return actor
    return None


def camera_blueprint(world, width, height, fov):
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(width))
    bp.set_attribute('image_size_y', str(height))
    bp.set_attribute('fov', str(fov))
    return bp


def run_cameras(args):
    """挂相机的主循环：栈起落 / 世界重载 / 自车消失都在这里自愈."""
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    quality = args.quality
    while True:
        cameras = []
        try:
            world = client.get_world()
            time.sleep(1.0)  # 同步模式下新客户端的第一份 episode state 要等一拍
            ego = find_ego(world)
            bp = camera_blueprint(world, args.width, args.height, args.fov)
            # 全图机位总是有（栈没起时至少能看见地图）
            overview = world.spawn_actor(bp, carla.Transform(
                carla.Location(*MAP_LOCATION), carla.Rotation(pitch=MAP_PITCH_DEG, yaw=0.0)))
            cameras.append(overview)
            overview.listen(lambda img: FRAMES.put('map', encode_jpeg(img, quality)))
            if ego is not None:
                chase = world.spawn_actor(bp, carla.Transform(
                    carla.Location(*CHASE_LOCATION), carla.Rotation(pitch=CHASE_PITCH_DEG)),
                    attach_to=ego)
                cameras.append(chase)
                chase.listen(lambda img: FRAMES.put('chase', encode_jpeg(img, quality)))
                top = world.spawn_actor(bp, carla.Transform(
                    carla.Location(z=TOP_HEIGHT_M), carla.Rotation(pitch=-89.9)),
                    attach_to=ego)
                cameras.append(top)
                top.listen(lambda img: FRAMES.put('top', encode_jpeg(img, quality)))
                FRAMES.status = f'追尾/俯视相机已挂到自车 id={ego.id}（{ego.type_id}）'
            else:
                FRAMES.status = '没找到自车（栈没起？）—— 只有全图；栈起后自动重挂'
            # 守望：帧不再更新（世界重载 / 栈收了 / 自车没了）→ 重来
            last_check = time.monotonic()
            while True:
                time.sleep(1.0)
                _, top_stamp = FRAMES.get('map')
                stale = time.monotonic() - top_stamp > 5.0
                if ego is None and find_ego(world) is not None:
                    FRAMES.status = '自车出现了，重挂追尾相机'
                    break
                if ego is not None and not ego.is_alive:
                    FRAMES.status = '自车没了，退回全图'
                    break
                if stale and time.monotonic() - last_check > 5.0:
                    FRAMES.status = '5 s 没有新帧（世界没在 tick 或已重载）—— 重连'
                    break
        except Exception as error:  # noqa: B902 —— 服务器崩了也别让直播进程死
            FRAMES.status = f'重连中：{error!r}'[:200]
            time.sleep(3.0)
        finally:
            for camera in cameras:
                try:
                    camera.stop()
                    camera.destroy()
                except Exception:  # noqa: B902
                    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--http-port', type=int, default=8080)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=540)
    parser.add_argument('--fov', type=float, default=90.0)
    parser.add_argument('--quality', type=int, default=80)
    args = parser.parse_args()
    try:
        import carla  # noqa: F401
    except ImportError:
        print('carla PythonAPI 不可用：本脚本只在云机上跑', file=sys.stderr)
        return 2
    threading.Thread(target=run_cameras, args=(args,), daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', args.http_port), Handler)
    print(f'直播就绪：http://127.0.0.1:{args.http_port}/（本机 ssh -L 转发后打开）', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
