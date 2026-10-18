FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-foxglove-bridge \
    ros-humble-tf2-ros-py \
    python3-colcon-common-extensions python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Operator UI bridge (separate layer so the big apt layer above stays cached)
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# evo: trajectory evaluation (ATE/RPE)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv
RUN uv pip install --system --no-cache "numpy<2" evo imageio-ffmpeg

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ws
