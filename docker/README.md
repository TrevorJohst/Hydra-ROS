# Docker Profiles

This folder contains multiple Docker configurations for building and running Hydra. 

## Profiles

- **`minimal`** (no GPU support):
  - ROS2 Jazzy
  - Basic ROS dev tools and C++ dependencies
- **`zed`**:
  - ROS 2 Jazzy
  - Basic ROS dev tools and C++ dependencies
  - CUDA 12.8
  - ZED SDK
  - TensorRT

## Quick Start
The following instructions will guide you through setting up and running Hydra using Docker with the `minimal` profile. You can replace `minimal` with `zed` in the commands below to use the ZED profile instead.

### Host
Before using Docker, make sure to:

1. Setup your workspace:

    ```shell
    mkdir -p ~/hydra_ws/src
    cd ~/hydra_ws
    echo "build: {cmake-args: [--no-warn-unused-cli, -DCMAKE_BUILD_TYPE=Release, -DCONFIG_UTILS_ENABLE_ROS=OFF]}" > colcon_defaults.yaml

    cd src
    git clone git@github.mit.edu:SPARK/Hydra-ROS.git hydra_ros
    vcs import . < hydra_ros/install/ros2.yaml
    ```

> :warning: **Warning**</br>
> In the `vcs import` step, GitHub may block too many concurrent requests. If you receive `kex_exchange_identification: read: Connection reset by peer` errors, try running `vcs import . < hydra/install/hydra.rosinstall --workers 1`.

2. You can skip this step if you are not running Hydra against a dataset. Download the ROS1 bag for the uhumans2 office scene [here](https://drive.google.com/file/d/1awAzQ7R1hdS5O1Z2zOcpYjK7F4_APq_p/view?usp=drive_link) and setup your dataset path (this only needs to be done once):

    ```shell
    cd ~/hydra_ws/src/hydra_ros/docker
    echo 'DATASETS_PATH=/path/to/your/datasets' > .env
    ```

> :information_source: **Note**</br>
> If running the minimal profile, you can run Hydra on the uhumans2 dataset. Download the ROS1 bag for the office scene [here](https://drive.google.com/file/d/1awAzQ7R1hdS5O1Z2zOcpYjK7F4_APq_p/view?usp=drive_link). The ROS1 bag will need to be converted to ROS2 bag (see below).

### Container
1. Build the image and run the container for the `minimal` profile:

```shell
cd ~/hydra_ws/src/hydra_ros/docker
make build PROFILE=minimal
make up PROFILE=minimal
make shell PROFILE=minimal
```

Once inside the container, you can build and run Hydra (you should already be in /root/hydra_ws when opening the shell):

```bash
colcon build --symlink-install --continue-on-error
source install/setup.bash
ros2 launch hydra_ros uhumans2.launch.yaml
```

> **:warning: Warning**<br> 
> If you encounter graphical issues (e.g. rviz not displaying), make sure you run `xhost +local:root` on the host machine and that `DISPLAY` is correctly set.

2. In a separate terminal, open another shell in the container:

```bash
cd ~/hydra_ws/src/hydra_ros/docker
make shell PROFILE=minimal
```

Before playing the bag, make sure to create an override for latching static tf topics, then play the bag:

```bash
echo "/tf_static: {depth: 1, durability: transient_local}" > ~/.tf_overrides.yaml
ros2 bag play /root/data/path/to/rosbag --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```

---

## Commands

The Makefile supports the following commands:

| Command       | Description                                            |
|---------------|--------------------------------------------------------|
| `make build`  | Builds the selected profile (must set `PROFILE=...`)  |
| `make run`    | Runs an interactive container (auto-removed)          |
| `make up`     | Runs container in background                          |
| `make shell`  | Opens a shell inside a running container              |
| `make stop`   | Stops a running container                             |
| `make start`  | Starts a stopped container                            |
| `make down`   | Stops and removes a container                         |
| `make clean`  | Prunes unused containers and images                   |

> :information_source: **Note**</br>
> If you don't want to type `PROFILE=...` every time, you can set the `PROFILE` environment variable in your shell via `export PROFILE=...`. This will make the Makefile use that profile by default.