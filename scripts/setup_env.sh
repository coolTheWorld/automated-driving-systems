#!/usr/bin/env bash
# =============================================================================
#  生成 .env —— 把宿主机的 UID/GID 和 GPU 设备组 GID 写给 docker compose
#
#  为什么需要这一步：
#    容器要访问 /dev/dri/renderD128，而内核判断权限只看**数字 GID**，不看组名。
#    宿主的 render 组 GID 因发行版而异（本机是 990，很多发行版是 104/108），
#    写死在 compose 里换台机器就失效。所以从宿主读出来注入。
#
#  用法：./scripts/setup_env.sh   （在仓库根目录执行）
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${REPO_ROOT}/.env"

VIDEO_GID="$(getent group video  | cut -d: -f3)"
RENDER_GID="$(getent group render | cut -d: -f3)"

if [[ -z "${VIDEO_GID}" || -z "${RENDER_GID}" ]]; then
  echo "错误：宿主机上找不到 video 或 render 组。" >&2
  echo "      这通常意味着没有可用的 GPU 渲染节点，环境 A 方案需重议。" >&2
  exit 1
fi

if [[ ! -e /dev/dri/renderD128 ]]; then
  echo "错误：/dev/dri/renderD128 不存在，无法做 GPU 直通。" >&2
  exit 1
fi

cat > "${ENV_FILE}" <<EOF
# 本文件由 scripts/setup_env.sh 自动生成，不要手改，也不要提交到 git。
HOST_UID=$(id -u)
HOST_GID=$(id -g)
VIDEO_GID=${VIDEO_GID}
RENDER_GID=${RENDER_GID}
DISPLAY=${DISPLAY:-:0}
WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
ROS_DOMAIN_ID=42
EOF

echo "已写入 ${ENV_FILE}："
sed 's/^/    /' "${ENV_FILE}"
echo
echo "下一步："
echo "    export COMPOSE_FILE=docker/docker-compose.local.yml"
echo "    docker compose build"
