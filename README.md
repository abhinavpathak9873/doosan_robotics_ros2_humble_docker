# Doosan ROS2 Docker Environment

A complete Docker-based development environment for Doosan robotics with ROS2 Humble, Intel RealSense camera support, and simulation capabilities.

## Overview

This containerized setup provides a ready-to-use environment for working with Doosan robotic arms, including:

- **ROS2 Humble** (full desktop installation)
- **Doosan Robot Package** (automatically cloned and built)
- **Intel RealSense SDK** and ROS2 wrapper
- **Gazebo/Ignition** simulation support
- **MoveIt** motion planning
- **Docker-in-Docker** support for running the Doosan emulator

## Prerequisites

### Host System Requirements

- Docker Engine (20.10+) and Docker Compose
- NVIDIA GPU drivers (optional, for hardware acceleration)

### Install Docker

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to the docker group
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt-get install docker-compose-plugin
```

### Enable X11 Display Access

```bash
xhost +local:docker
```

## Quick Start

### 1. Clone or Create Project Directory

```bash
mkdir -p ~/doosan-ros2-project
cd ~/doosan-ros2-project
```

Place the `Dockerfile`, `docker-compose.yml`, and `container-entrypoint.sh` files in this directory.

### 2. Build the Container

```bash
docker-compose build
```

### 3. Start the Container

```bash
docker-compose up -d
```

### 4. Enter the Container

```bash
docker exec -it doosan-ros2-humble bash
```

On first run, the entrypoint script will:
- Clone the doosan-robot2 repository
- Initialize and update rosdep
- Install all ROS dependencies
- Build the workspace with colcon
- Attempt to install the Doosan emulator (if Docker socket is available)

This process may take 10-15 minutes on the first run.

## Usage

### Launch Virtual Robot with RViz

The easiest way to test the setup:

```bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual model:=m1013
```

### Launch Real Robot with RViz

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.50.100 port:=12345 model:=m1013
```

### Launch Virtual Robot with Gazebo

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 x:=0 y:=0
```

### Launch Real Robot with Gazebo

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=real host:=192.168.50.100 model:=m1013
```

### Launch with MoveIt2 (Virtual Mode)

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1013 host:=127.0.0.1
```
### Launch with MoveIt2 (Real Mode)
⚠ Caution: MoveIt2 requires Controller Version 2.12 or higher. Check the controller versionon the pendant and update to the latest version from the Doosan's support website. 
```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m1013 host:=192.168.50.100
```

### Launch RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py
```
### Launch RealSense Camera with PointCloud Enabled

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_pointcloud:=true \
  pointcloud.enable:=true \
  align_depth.enable:=true
```

### Check RealSense Camera Detection and Find Camera's Serial Number 

```bash
rs-enumerate-devices 
```

### Launch Multiple RealSense Cameras

```bash
# Launch multiple cameras from config
ros2 launch realsense2_camera rs_multi_camera_launch.py

# With custom serial numbers (2 cameras)
ros2 launch realsense2_camera rs_multi_camera_launch.py \
  serial_no_camera1:=<SERIAL_1> \
  serial_no_camera2:=<SERIAL_2>

# With 3 cameras
ros2 launch realsense2_camera rs_multi_camera_launch.py \
  serial_no_camera1:=<SERIAL_1> \
  serial_no_camera2:=<SERIAL_2> \
  serial_no_camera3:=<SERIAL_3>
```

### Launch Multiple RealSense Cameras with PointCloud Enabled 

```bash
# All cameras with point clouds enabled
ros2 launch realsense2_camera rs_multi_camera_launch.py \
  serial_no_camera1:=<SERIAL_1> \
  serial_no_camera2:=<SERIAL_2> \
  pointcloud.enable:=true \
  align_depth.enable:=true
```


## Directory Structure

```
doosan-ros2-project/
├── Dockerfile                 # Container image definition
├── docker-compose.yml         # Service configuration
├── container-entrypoint.sh    # Startup script
├── doosan_workspace/          # ROS2 workspace (persisted on host)
│   ├── src/                   # Source packages
│   │   └── doosan-robot2/     # Auto-cloned Doosan package
│   ├── build/                 # Build artifacts
│   └── install/               # Installed packages
├── doosan_config/             # Configuration files (mapped to /root/.config)
└── shared/                    # Shared files (mapped to /root/shared)
```

## Useful Commands

### Container Management

```bash
# Start container
docker-compose up -d

# Stop container
docker-compose down

# Restart container
docker-compose restart

# View logs
docker-compose logs -f

# Enter container shell
docker exec -it doosan-ros2-humble bash
```

### Inside Container

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# List available ROS2 nodes
ros2 node list

# List available ROS2 topics
ros2 topic list

# Rebuild workspace
cd /ros2_ws
colcon build --symlink-install

# Update Doosan repository
cd /ros2_ws/src/doosan-robot2
git pull
cd /ros2_ws
colcon build --symlink-install
```

## Troubleshooting

### Container Won't Start

Check Docker logs:
```bash
docker-compose logs doosan-ros2
```

### Display Issues (GUI Not Showing)

1. Ensure X11 forwarding is enabled:
   ```bash
   # On host
   xhost +local:docker
   ```

2. Check DISPLAY variable:
   ```bash
   # Inside container
   echo $DISPLAY
   ```

3. Test with a simple GUI app:
   ```bash
   # Inside container
   xclock
   ```

### Permissions Issues

The container runs as root for maximum compatibility with devices and Docker socket access. If you need to change files on the host that are owned by root:

```bash
# On host
sudo chown -R $USER:$USER ./doosan_workspace
```

## Network Configuration

The container uses `network_mode: host`, which means:
- The container shares the host's network stack
- ROS2 discovery works seamlessly across host and container
- You can access services on `localhost`
- Port mappings are not needed

