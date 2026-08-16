#!/bin/bash
# =============================================================================
#  cloud_window_open.sh —— 租机窗口一键开场（Vast.ai 实例宿主侧，root）
#
#  用法（本机）：
#      scp -P <port> scripts/cloud_window_open.sh root@<host>:/root/ && \
#      rsync -az --delete -e "ssh -p <port>" --exclude .git --exclude build \
#            --exclude install --exclude log --exclude "core.*" ./ root@<host>:/root/ads/ && \
#      ssh -p <port> root@<host> 'setsid nohup bash /root/cloud_window_open.sh > /root/open.out 2>&1 &'
#      # 之后 grep -E "里程碑|FATAL" /root/open.out 看进度；全部里程碑过 = 可以派轮
#
#  幂等：每一步先查后做，重跑只补缺的。全程不需要交互。
#  可选：export CARLA_SHA256=<可信下载的 sha256> 后再起 —— 脚本会强校验 tarball；未设时只打印实测值。
#
#  ## 它把三个窗口里手工踩过的坑一次焊死（每条都有实测出处）
#   1. 驱动：580-open × UE4.26 = Xid 32 秒崩（窗口 2，4090）；5090+580-open 却能跑
#      （窗口 3）—— 宿主特异。所以**不自动降级**（降级要 reboot、要用户拍板），
#      只在服务端冒烟失败且驱动是 580-open 时指名报错并给出降级命令。
#   2. CARLA 下载：多源 + 断点续传（7.9 GB，约 20 分钟）；解包后 carla 用户要能读
#      （UE4 拒绝 root 运行）；carla 用户要进 video/render 组。
#   3. 容器：镜像本体不含 pip；carla wheel 要在容器里装（窗口 2 两轮
#      BRINGUP_TIMEOUT 才追到「No module named carla」）。
#   4. dpkg 锁：unattended-upgrades 会抢锁让 apt 静默失败 —— 开场先掐掉。
#   5. 服务端冒烟是最终裁判：Vulkan 枚举过了不等于能跑（枚举不提交命令流）。
# =============================================================================
set -u
M() { echo "[$(date +%H:%M:%S)] $*"; }
export DEBIAN_FRONTEND=noninteractive
ADS=/root/ads
CARLA_DIR=/root/carla

M "══ 0. 宿主体检 ══"
DRV=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
M "GPU=${GPU:-无} 驱动=${DRV:-无} 核数=$(nproc) 内存=$(free -g | awk 'NR==2{print $2}')G 盘剩=$(df -h / | awk 'NR==2{print $4}')"
[ -n "$DRV" ] || { M "FATAL: nvidia-smi 不可用（宿主置备坏了 —— 换宿主，不要同宿主重建）"; exit 1; }
if dpkg -l 2>/dev/null | grep -q "nvidia-driver-580-open"; then
  M "⚠️ 驱动是 580-open：4090 上实测 Xid 32 秒崩、5090 上能跑 —— 以下面的服务端冒烟为准"
fi

M "══ 1. 掐掉抢 dpkg 锁的 + 基础包 ══"
systemctl stop --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true
for i in $(seq 1 30); do fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || break; sleep 5; done
apt-get update -qq >/dev/null 2>&1 || true
apt-get install -y -qq rsync libomp5 libvulkan1 vulkan-tools xdg-user-dirs psmisc >/dev/null 2>&1 || true

M "══ 2. carla 用户（UE4 拒绝 root）══"
id carla >/dev/null 2>&1 || useradd -m -s /bin/bash carla
usermod -aG video,render carla 2>/dev/null || true

M "══ 3. CARLA 0.9.16 ══"
if [ -x ${CARLA_DIR}/CarlaUE4.sh ]; then
  M "已存在，跳过下载"
else
  mkdir -p ${CARLA_DIR}; cd /root
  for URL in \
    "https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.16.tar.gz" \
    "https://tiny.carla.org/carla-0-9-16-linux"; do
    M "下载 $URL（7.9 GB，断点续传）"
    curl -fL -C - -sS -o CARLA_0.9.16.tar.gz --connect-timeout 20 "$URL" && break
  done
  # 完整性：CARLA_SHA256 已知时强校验（不匹配即停），未知时把实测值印出来供下窗口钉住
  # （2026-08-16 安全复审：8 GB tarball 走镜像/短链无校验就解包运行）。
  ACTUAL_SHA=$(sha256sum CARLA_0.9.16.tar.gz | awk '{print $1}')
  M "CARLA_0.9.16.tar.gz sha256 = ${ACTUAL_SHA}"
  if [ -n "${CARLA_SHA256:-}" ] && [ "${ACTUAL_SHA}" != "${CARLA_SHA256}" ]; then
    M "FATAL: CARLA tarball sha256 与钉住的值不符（期望 ${CARLA_SHA256}）—— 下载源被换了或文件残缺"
    exit 6
  fi
  [ -s CARLA_0.9.16.tar.gz ] || { M "FATAL: CARLA 下载失败"; exit 2; }
  tar -xzf CARLA_0.9.16.tar.gz -C ${CARLA_DIR} && rm -f CARLA_0.9.16.tar.gz
fi
# carla 用户只需要**穿过** /root 到 ${CARLA_DIR}：只给 /root 加 o+x（不递归、不给读），
# CARLA 目录整体归 carla 用户。此前的 `chmod -R o+rX /root` 把 /root/.ssh、.docker/config.json、
# bash_history、/root/ads 全设成其他用户可读（2026-08-16 安全复审 Medium）。
chmod o+x /root 2>/dev/null || true
chown -R carla:carla ${CARLA_DIR} 2>/dev/null || chmod -R o+rX ${CARLA_DIR}
M "里程碑①：CARLA 就绪"

M "══ 4. 容器（docker compose cloud）══"
[ -d ${ADS}/docker ] || { M "FATAL: ${ADS} 没有仓库 —— 先 rsync"; exit 3; }
cd ${ADS}
if ! docker ps --format '{{.Names}}' | grep -q '^ads-dev$'; then
  docker compose -f docker/docker-compose.cloud.yml up -d --build > /root/build.out 2>&1 || \
    { M "FATAL: compose 构建失败，查 /root/build.out"; exit 4; }
fi
docker ps --format '{{.Names}}' | grep -q '^ads-dev$' || { M "FATAL: 容器没起来"; exit 4; }
M "里程碑②：容器已起"

M "══ 5. 容器内 pip + carla wheel（镜像本体不含）══"
docker exec ads-dev python3 -c "import carla" >/dev/null 2>&1 || \
  docker exec -u root ads-dev bash -c "apt-get update -qq >/dev/null 2>&1; apt-get install -y -qq python3-pip >/dev/null 2>&1; python3 -m pip install -q carla==0.9.16 --break-system-packages 2>&1 | tail -1"
docker exec ads-dev python3 -c "import carla" >/dev/null 2>&1 || { M "FATAL: carla wheel 装不上"; exit 5; }
M "里程碑③：carla wheel OK"

M "══ 6. 容器内 colcon build ══"
docker exec ads-dev bash -c 'set +u; source /opt/ros/jazzy/setup.bash; set -u; cd /workspace && MAKEFLAGS=-j8 colcon build --symlink-install --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release > /workspace/.build.log 2>&1'
docker exec ads-dev bash -c 'ls /workspace/install/setup.bash' >/dev/null 2>&1 || { M "FATAL: colcon 失败，查 /workspace/.build.log"; exit 6; }
M "里程碑④：colcon 完成"

M "══ 7. 服务端冒烟（最终裁判）══"
su - carla -c "pkill -9 -f CarlaUE4" 2>/dev/null; sleep 2
su - carla -c "cd ${CARLA_DIR} && unset DISPLAY && setsid nohup ./CarlaUE4.sh -RenderOffScreen -quality-level=Low -carla-rpc-port=2000 > /tmp/carla.log 2>&1 < /dev/null &"
for i in $(seq 1 60); do
  timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null && break
  if grep -q "Segmentation fault" /tmp/carla.log 2>/dev/null; then
    M "FATAL: 服务端秒崩（$(grep -m1 -oE 'VK_ERROR_[A-Z_]+' /tmp/carla.log 2>/dev/null || echo 未知)）。"
    M "  若驱动是 580-open：降 550 专有版（需 reboot，用户拍板）——"
    M "  apt-get remove -y nvidia-driver-580-open nvidia-dkms-580-open nvidia-driver-pinning-580 && apt-get install -y nvidia-driver-550 && dpkg -l nvidia-driver-550 | grep -q ^ii && reboot"
    exit 7
  fi
  sleep 5
done
timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null || { M "FATAL: 服务端 5 分钟未监听，查 /tmp/carla.log"; exit 7; }
M "里程碑⑤：服务端监听 :2000 —— 窗口开场完成，可以派轮（l3c_*_round.sh）"
