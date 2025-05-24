# ros2-dev/Dockerfile
#notes:
# - run this on host machine: xhost +local:root

FROM osrf/ros:jazzy-desktop

#required to skip interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

#update + install dev tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    git \
    curl \
    wget \
    tmux \
    vim \
    net-tools \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

#hydra dependencies
RUN apt-get update && apt-get install -y \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libcli11-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

#install pipx (for rosbags)
RUN apt-get update && apt-get install -y pipx && \
    pipx install rosbags && \
    echo 'export PATH=$PATH:/root/.local/bin' >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*

#copy source to a temp directory for rosdep
# COPY hydra_ws/src/ /tmp/deps/src/
COPY ../../../src/ /tmp/deps/src/

#run rosdep install on the copied packages
RUN apt-get update && rosdep update && \
    rosdep install --from-paths /tmp/deps/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

#setup ros2 workspace (mounted from host)
WORKDIR /root/hydra_ws

#run source ros2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

#alias to install dependecies (shouldn't be needed)
RUN echo "alias setup_deps='cd /root/hydra_ws && sudo apt update && rosdep install --from-paths src --ignore-src -r -y'" >> /root/.bashrc

#alias to build workspace
RUN echo "alias cb='cd /root/hydra_ws && colcon build --symlink-install --continue-on-error'" >> /root/.bashrc