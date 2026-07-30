#!/usr/bin/env bash
# =============================================================================
#  P0a 任务 1.4 —— go / no-go 验证：容器内能否拿到 OpenGL 硬件加速
#
#  整个「环境 A：本机 Gazebo」方案压在这个假设上。失败则需回到 SPEC §4.1 重议，
#  **不要用软件渲染硬撑** —— llvmpipe 下 Gazebo 帧率个位数，那不是能开发的环境。
#
#  在**容器内**执行：
#      docker compose exec dev /workspace/scripts/verify_gpu.sh
# =============================================================================
set -uo pipefail

PASS=0
fail() { echo "  ✗ $1"; PASS=1; }
ok()   { echo "  ✓ $1"; }

echo "=============================================="
echo " P0a 1.4  容器内 GPU 硬件加速验证"
echo "=============================================="

echo
echo "[1/4] GPU 设备节点"
# WSL2 的 GPU 通路是 dxgkrnl 的 /dev/dxg，不是 /dev/dri。
# /dev/dri 也在，但只是兼容外观 —— 只映射 /dev/dri 会静默退化成 llvmpipe。
if [[ -e /dev/dxg ]]; then
  ok "/dev/dxg 存在（WSL2 硬件加速的实际通路）"
else
  fail "/dev/dxg 不存在 —— compose 的 devices 里漏了 /dev/dxg:/dev/dxg"
  echo "     症状极具迷惑性：不报任何错，直接变软件渲染。"
fi
if [[ -e /dev/dri/renderD128 ]]; then
  ok "/dev/dri/renderD128 存在"
  if [[ -r /dev/dri/renderD128 && -w /dev/dri/renderD128 ]]; then
    ok "当前用户可读写（group_add 的 GID 注入生效）"
  else
    fail "无读写权限 —— 检查 .env 里的 RENDER_GID 是否与宿主一致"
    echo "     宿主执行 \`getent group render\` 取正确 GID，再跑 scripts/setup_env.sh"
  fi
else
  fail "/dev/dri/renderD128 不存在 —— compose 的 devices 映射未生效"
fi

echo
echo "[2/4] Mesa d3d12 驱动与 dxcore 库"
if ls /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so >/dev/null 2>&1; then
  ok "d3d12_dri.so 已安装"
else
  fail "缺少 d3d12_dri.so —— 检查镜像是否装了 libgl1-mesa-dri"
fi
if [[ -e /usr/lib/wsl/lib/libdxcore.so ]]; then
  ok "libdxcore.so 已挂载"
  # d3d12_dri.so 是 dlopen 加载它的，所以必须在库搜索路径里
  if [[ ":${LD_LIBRARY_PATH:-}:" == *":/usr/lib/wsl/lib:"* ]]; then
    ok "LD_LIBRARY_PATH 已包含 /usr/lib/wsl/lib"
  else
    fail "LD_LIBRARY_PATH 未包含 /usr/lib/wsl/lib —— 会导致静默退化为软件渲染"
  fi
else
  fail "libdxcore.so 未挂载 —— 检查 compose 是否挂了 /usr/lib/wsl"
fi
# Ubuntu 24.04 的 Mesa 25.2.8 不会自动选中 d3d12（宿主 26.04 的 Mesa 26.0.3 会）。
# Jazzy 绑死 24.04，升不了 Mesa，所以只能显式指定。
if [[ "${GALLIUM_DRIVER:-}" == "d3d12" ]]; then
  ok "GALLIUM_DRIVER=d3d12 已设置"
else
  fail "GALLIUM_DRIVER 未设为 d3d12（当前：'${GALLIUM_DRIVER:-未设置}'）"
  echo "     24.04 的 Mesa 自动探测不到 WSL d3d12 设备，不显式指定就会落到 llvmpipe。"
fi

echo
echo "[3/4] OpenGL 实际渲染器（决定性判据）"
if ! command -v glxinfo >/dev/null 2>&1; then
  fail "glxinfo 不存在 —— 镜像缺 mesa-utils"
else
  GLX="$(glxinfo -B 2>/dev/null)"
  echo "$GLX" | sed -n '/Extended renderer info/,/Max core profile/p' | sed 's/^/     /'

  RENDERER="$(echo "$GLX" | grep -iE '^\s*(Device|OpenGL renderer string)' | head -1)"
  ACCEL="$(echo "$GLX"    | grep -iE 'Accelerated:'                        | head -1)"

  if echo "$ACCEL" | grep -qi 'yes'; then
    ok "Accelerated: yes"
  else
    fail "未获得硬件加速（$ACCEL）"
  fi

  if echo "$RENDERER" | grep -qiE 'llvmpipe|softpipe|swrast'; then
    fail "渲染器是软件光栅化：$RENDERER"
    echo "     这是最需要警惕的情况：程序能跑，但慢到无法开发。"
  elif echo "$RENDERER" | grep -qiE 'd3d12|radeon'; then
    ok "渲染器为硬件设备：$(echo "$RENDERER" | xargs)"
  else
    fail "渲染器无法识别：$RENDERER"
  fi
fi

echo
echo "[4/4] OpenGL 版本（Gazebo 的 OGRE 需要 ≥ 3.3）"
GLVER="$(glxinfo -B 2>/dev/null | grep -i 'Max core profile version' | grep -oE '[0-9]+\.[0-9]+' | head -1)"
if [[ -n "${GLVER}" ]]; then
  if awk -v v="${GLVER}" 'BEGIN{exit !(v+0 >= 3.3)}'; then
    ok "OpenGL core ${GLVER} ≥ 3.3"
  else
    fail "OpenGL core ${GLVER} < 3.3，Gazebo 无法运行"
  fi
else
  fail "读不到 OpenGL 版本"
fi

echo
echo "=============================================="
if [[ ${PASS} -eq 0 ]]; then
  echo " 结果：通过 ✓   可以进入 S2（装 Gazebo Harmonic）"
else
  echo " 结果：失败 ✗   停在这里，不要继续 S2。"
  echo "        按上面的提示排查；若都无效，回到 SPEC §4.1 重议环境 A。"
fi
echo "=============================================="
exit ${PASS}
