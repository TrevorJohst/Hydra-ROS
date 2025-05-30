# Docker Profiles

This directory contains multiple Docker configurations for building and running Hydra. 

## Profiles

- **`minimal`** (no GPU support):
  - ROS 2 Jazzy
  - Basic ROS dev tools and C++ dependencies
- **`a1`**:
  - ROS 2 Jazzy
  - Basic ROS dev tools and C++ dependencies
  - CUDA 12.8
  - TensorRT
- **`zed`**:
  - ROS 2 Jazzy
  - Basic ROS dev tools and C++ dependencies
  - CUDA 12.8
  - ZED SDK
  - TensorRT

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


> :grey_exclamation: **Note**</br>
> You must specify the profile you want to use by setting the `PROFILE` variable when running the commands (e.g., `make build PROFILE=minimal`). If you don't want to type `PROFILE=<profile>` every time, you can set the `PROFILE` environment variable in your shell via `export PROFILE=<profile>`. This will make the Makefile use that profile by default.

---

## Quick Start (minimal)
The following instructions will guide you through setting up and running Hydra using Docker with the `minimal` profile.

Before starting, export the `WORKSPACE` environment variable to point to your Hydra workspace directory (e.g., `export WORKSPACE=/path/to/hydra_ws`). This is only needed for copy/pasting the commands in the quick start.

### Host (minimal)
Before using Docker, make sure to:

1. Setup your workspace:

    ```shell
    mkdir -p $WORKSPACE/src
    cd $WORKSPACE
    echo "build: {cmake-args: [--no-warn-unused-cli, -DCMAKE_BUILD_TYPE=Release, -DCONFIG_UTILS_ENABLE_ROS=OFF]}" > colcon_defaults.yaml

    cd src
    git clone git@github.mit.edu:SPARK/Hydra-ROS.git hydra_ros
    vcs import . < hydra_ros/install/ros2.yaml
    ```

> :warning: **Warning**</br>
> In the `vcs import` step, GitHub may block too many concurrent requests. If you receive `kex_exchange_identification: read: Connection reset by peer` errors, try running `vcs import . < hydra/install/hydra.rosinstall --workers 1`.

2. You can skip this step if you are not running Hydra against a dataset; otherwise, setup your dataset path (this only needs to be done once):

    ```shell
    cd $WORKSPACE/src/hydra_ros/docker
    grep -q '^DATASETS_PATH=' .env || echo 'DATASETS_PATH=/path/to/your/datasets' >> .env
    ```

3. If running the minimal profile, you can run Hydra on the uhumans2 dataset. Download the ROS 1 bag for the office scene [here](https://drive.google.com/file/d/1awAzQ7R1hdS5O1Z2zOcpYjK7F4_APq_p/view?usp=drive_link). The ROS 1 bag will need to be converted to ROS 2 bag (see below).

### Container (minimal)
1. Build the image and run the container for the `minimal` profile:

```shell
cd $WORKSPACE/src/hydra_ros/docker
make build PROFILE=minimal
make up PROFILE=minimal
make shell PROFILE=minimal
```

Once inside the container, you can build and run Hydra (you should already be in `/root/hydra_ws` when opening the shell):

```bash
colcon build --symlink-install --continue-on-error
source install/setup.bash
ros2 launch hydra_ros uhumans2.launch.yaml
```


> **:warning: Warning**<br> 
> If you encounter graphical issues (e.g. rviz not displaying), make sure you run `xhost +local:root` on the host machine and that `DISPLAY` is correctly set.

2. In a separate terminal, open another shell in the container:

```bash
cd $WORKSPACE/src/hydra_ros/docker
make shell PROFILE=minimal
```

Before playing the bag, make sure to create an override for latching static tf topics, then play the bag:

```bash
echo "/tf_static: {depth: 1, durability: transient_local}" > ~/.tf_overrides.yaml
ros2 bag play /root/data/path/to/rosbag --clock --qos-profile-overrides-path ~/.tf_overrides.yaml
```


> **:warning: Warning**<br> 
> You must convert the ROS 1 bag to a ROS 2 bag before playing it. The `rosbags-convert` tool is preinstalled in the container, and you can use it to convert the bag using the following command: `rosbags-convert --src path/to/office.bag --dst path/to/office`. You should run this in the container if you don't have `rosbags-convert` installed on your host machine.

## Quick Start (zed)

### Host (zed)
You can repeat the steps above using the `zed` profile instead of `minimal`, but you must complete a few additional steps on the host to run with hardware. 

1. Add the `zed-ros2-wrapper` to your workspace (the dependencies will be installed via the dockerfile):

```shell
cd $WORKSPACE/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
```

2. While the uhumans2 dataset has pre-segmented images, you must run semantic segmentation on the images. Download the default [pretrained model](https://drive.google.com/file/d/1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-/view?usp=drive_link) to the directory `$WORKSPACE/.models/`.

3. (optional) To avoid re-optimizing the model when running the container, set the `ZED_CACHE` environment variable to mount a host directory for the zed cache:

```shell
mkdir -p "$WORKSPACE/.zed_cache"

cd $WORKSPACE/src/hydra_ros/docker
grep -q '^ZED_CACHE=' .env || echo "ZED_CACHE=$WORKSPACE/.zed_cache" >> .env
```
### Container (zed)

Once inside the container, you can build and run Hydra for the zed profile (you should already be in `/root/hydra_ws` when opening the shell):

```shell
colcon build --symlink-install --continue-on-error
source install/setup.bash
ros2 launch hydra_ros hydra_zed.launch.yaml
```

## Quick Start (a1)
To run with a bag recorded on the A1 sensor payload (e.g., D455/T265), you can use the `a1` profile. You can simply run Hydra along with the bag.

1. Run the `A1` launch script for Hydra:
```bash
ros2 launch hydra_ros a1.launch.yaml use_sim_time:=true
```

2. Run the bag:
```bash
ros2 bag play /path/to/bag --clock --exclude-topics /tf_static
```
> :grey_exclamation: **Note**</br>
> The static tfs are included in the launch script, so you should exclude them when playing the bag.
