# syntax=docker/dockerfile:1
# =============================================================================
#  园区自动驾驶开发镜像 —— 本地 Gazebo 与云端 CARLA 共用同一份
#
#  版本依据 SPEC.md §4「版本约束链」：
#      Gazebo Harmonic ──官方组合──▶ ROS 2 Jazzy ──▶ Ubuntu 24.04
#
#  ⚠️ 不要改 ROS 版本。Jazzy + Harmonic 是官方组合；换 Humble 会退回非官方组合，
#     换 Lyrical 则两侧生态皆空白。理由见 SPEC §9 D3。
# =============================================================================
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# -----------------------------------------------------------------------------
# 1. 图形栈 —— P0a 任务 1.4 的 go/no-go 就取决于这一层
# -----------------------------------------------------------------------------
# WSL2 下没有真正的 Linux GPU 驱动，走的是这条链路：
#
#     容器内 OpenGL 调用
#         → Mesa 的 d3d12 gallium 驱动 (libgl1-mesa-dri 里的 d3d12_dri.so)
#         → libdxcore.so (来自宿主 /usr/lib/wsl/lib，由 compose 挂载)
#         → /dev/dri  (WSL 的 dxgkrnl)
#         → Windows 侧的 Radeon 780M
#
# 关键细节：d3d12_dri.so 是用 **dlopen** 加载 libdxcore.so 的，没有 DT_NEEDED 记录，
# 所以链接器不会自动去找它 —— 必须靠 LD_LIBRARY_PATH 显式指路。
# 这一点做错的表现极具迷惑性：容器能起、rviz2 能开，但悄悄退化成 llvmpipe 软件渲染，
# 帧率个位数，而你会以为是 Gazebo 配置问题。所以任务 1.4 要显式验证 Renderer 字符串。
RUN apt-get update && apt-get install -y --no-install-recommends \
        mesa-utils \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libegl-mesa0 \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2. ROS 2 桌面组件
# -----------------------------------------------------------------------------
# rviz2 / rqt / tf2_tools 都在 desktop 元包里，SPEC §6 的调试命令依赖它们。
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-jazzy-tf2-tools \
        ros-jazzy-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 3. C++ 工具链与数学库
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build gdb \
        git vim less tmux curl \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
        clang-format clang-tidy \
        libeigen3-dev \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 4. DDS 锁定 —— SPEC §4「不可自行改动的三条」第 3 条
# -----------------------------------------------------------------------------
# CARLA 0.9.16 的原生 ROS 2 接口内嵌 Fast DDS，**只支持 Fast DDS**。
# 换成 CycloneDDS 的症状是：本地 Gazebo 一切正常，一上 CARLA 完全收不到数据。
# 这种「一半环境好使」的故障最难查，所以在镜像层面锁死。
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# -----------------------------------------------------------------------------
# 5. 非 root 用户
# -----------------------------------------------------------------------------
# 若以 root 运行，容器写进挂载卷的文件在宿主上属主会变成 root，
# 之后你在 WSL 里改自己的代码都要 sudo。UID/GID 与宿主对齐可避免。
#
# 注意 Ubuntu 24.04 基础镜像自带一个 UID 1000 的 `ubuntu` 用户，
# 直接 useradd -u 1000 会冲突，所以先清掉占位者。
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN if getent passwd ${UID} >/dev/null; then \
        userdel -r "$(getent passwd ${UID} | cut -d: -f1)" 2>/dev/null || true; \
    fi \
 && if ! getent group ${GID} >/dev/null; then groupadd -g ${GID} ${USERNAME}; fi \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME} \
 && chmod 0440 /etc/sudoers.d/90-${USERNAME}

# GPU 设备组（video / render）**不在这里加**。
# 宿主的 render 组 GID 是 990，而容器内同名组的 GID 不同 —— 按名字加没用，
# 内核只认数字 GID。所以改由 compose 的 group_add 以数字形式注入，
# 换机器时只改 .env 即可，不用重建镜像。

# -----------------------------------------------------------------------------
# 6. shell 环境
# -----------------------------------------------------------------------------
RUN { \
      echo 'source /opt/ros/jazzy/setup.bash'; \
      echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash'; \
      echo 'export PS1="\[\e[1;32m\]ads\[\e[0m\]:\w\$ "'; \
    } >> /home/${USERNAME}/.bashrc \
 && chown ${UID}:${GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace

# =============================================================================
#  后续阶段将在此追加（现在不装，是为了让 S1 的 go/no-go 尽快出结果）：
#    S2   ros-jazzy-ros-gz        Jazzy + Harmonic 官方组合，会自动带上 gz-harmonic
#    S3   ros-jazzy-pcl-ros / libpcl-dev
#    P0b  CARLA 0.9.16 PythonAPI wheel（Python 3.12）
# =============================================================================
