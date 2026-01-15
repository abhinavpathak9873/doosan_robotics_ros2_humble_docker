#!/bin/bash
echo "🔥 container-entrypoint.sh is running 🔥"
sleep 2
set -euo pipefail

# ---------- Config ----------
ROS_DISTRO=${ROS_DISTRO:-humble}
ROS2_WS=${ROS2_WS:-/ros2_ws}
MARKER="${ROS2_WS}/.container_setup_done"
REALSENSE_ENV_FILE="/etc/profile.d/librealsense_env.sh"
ROS2_ENV_FILE="/etc/profile.d/ros2_env.sh"

# ---------- Helper ----------
log() { printf '\e[1;34m• %s\e[0m\n' "$*"; }

echo ""
log "Doosan ROS2 container bootstrap (running as root)"
log "Workspace: ${ROS2_WS}"
log "ROS distro: ${ROS_DISTRO}"
echo ""

# Ensure runtime dir exists
mkdir -p "${ROS2_WS}/src"
mkdir -p /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root
chown -R root:root /tmp/runtime-root || true

# Persist RealSense RSUSB backend & logging for all shells
if [ ! -f "${REALSENSE_ENV_FILE}" ]; then
  cat <<EOF > "${REALSENSE_ENV_FILE}"
# Librealsense runtime backend (RSUSB recommended for containers)
export LIBREALSENSE_USB_BACKEND=RSUSB
export LRS_LOG_LEVEL=INFO
EOF
  chmod 0644 "${REALSENSE_ENV_FILE}" || true
  log "Wrote ${REALSENSE_ENV_FILE}"
fi

# Persist ROS2 environment variables and useful aliases
if [ ! -f "${ROS2_ENV_FILE}" ]; then
  cat <<EOF > "${ROS2_ENV_FILE}"
# ROS2 environment
export ROS_DISTRO=${ROS_DISTRO}
export ROS2_WS=${ROS2_WS}
export HOME=/root
# Convenience aliases
alias doosan-virtual='ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual model:=m1013'
alias realsense-launch='ros2 launch realsense2_camera rs_launch.py'
EOF
  chmod 0644 "${ROS2_ENV_FILE}" || true
  log "Wrote ${ROS2_ENV_FILE}"
fi

# Source ROS system install for this script context
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  . /opt/ros/${ROS_DISTRO}/setup.bash
  log "Sourced /opt/ros/${ROS_DISTRO}/setup.bash"
else
  log "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found"
fi

# Early export for runtime checks (this ensures rs-enumerate-devices sees RSUSB)
export LIBREALSENSE_USB_BACKEND=${LIBREALSENSE_USB_BACKEND:-RSUSB}
export LRS_LOG_LEVEL=${LRS_LOG_LEVEL:-INFO}

# Clone doosan repo if missing
if [ ! -d "${ROS2_WS}/src/doosan-robot2/.git" ]; then
  log "Cloning doosan-robot2..."
  mkdir -p "${ROS2_WS}/src"
  git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git "${ROS2_WS}/src/doosan-robot2" || {
    log "Warning: git clone failed (continuing)"
  }
else
  log "doosan-robot2 already present; skipping clone"
fi
echo ""

# Initialize & update rosdep (idempotent)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  log "Initializing rosdep..."
  rosdep init || true
fi
log "Updating rosdep (this may take a minute)..."
rosdep update || true
echo ""

# Install package dependencies (skip librealsense2 to avoid DKMS dependency)
log "Installing rosdep dependencies (skipping librealsense2)..."
cd "${ROS2_WS}"
. /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" --skip-keys=librealsense2 -y || {
  log "rosdep install returned non-zero (continuing)"
}
echo ""

# Run install_emulator.sh if available (best-effort; requires docker socket)
EMULATOR_SCRIPT="${ROS2_WS}/src/doosan-robot2/install_emulator.sh"
if [ -f "${EMULATOR_SCRIPT}" ]; then
  log "Found install_emulator.sh — attempting to run (requires /var/run/docker.sock & privileged)"
  pushd "${ROS2_WS}/src/doosan-robot2" >/dev/null
  chmod +x ./install_emulator.sh || true
  if command -v docker >/dev/null 2>&1 && docker ps >/dev/null 2>&1; then
    ./install_emulator.sh || log "install_emulator.sh exited non-zero (continuing)"
  else
    log "Docker CLI/socket not available from inside container; skipping emulator install."
  fi
  popd >/dev/null
else
  log "install_emulator.sh not present; skipping emulator install."
fi
echo ""

# Build workspace: try full parallel build, retry with safer flags if it fails
log "Building ROS2 workspace with colcon (this may take long)..."
cd "${ROS2_WS}"
. /opt/ros/${ROS_DISTRO}/setup.bash

# If an old install exists, source it so dependent packages build reliably
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  . "${ROS2_WS}/install/setup.bash" || true
  log "Sourced existing workspace install/setup.bash"
fi

# Primary attempt: parallel workers
if colcon build --parallel-workers "$(nproc)" --symlink-install; then
  log "colcon build succeeded (parallel)"
else
  log "colcon parallel build failed; retrying with single-run --symlink-install"
  if colcon build --symlink-install; then
    log "colcon build succeeded on retry"
  else
    log "colcon build failed on retry — continuing but environment may be incomplete"
  fi
fi

# Ensure install/setup.bash exists for shells — create lightweight fallback if not
if [ ! -f "${ROS2_WS}/install/setup.bash" ]; then
  log "Workspace install/setup.bash not found — creating lightweight fallback to source /opt/ros"
  mkdir -p "${ROS2_WS}/install"
  cat <<EOF > "${ROS2_WS}/install/setup.bash"
#!/bin/bash
# Fallback workspace setup (no packages installed), sources system ROS
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  . /opt/ros/${ROS_DISTRO}/setup.bash
fi
EOF
  chmod +x "${ROS2_WS}/install/setup.bash"
fi

# Mark completion (idempotent)
touch "${MARKER}" || true
log "Bootstrap complete — marker: ${MARKER}"
echo ""

# Final environment note: write to /root/.bashrc so interactive shells load everything quickly
BASHRC="/root/.bashrc"
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" "${BASHRC}" 2>/dev/null; then
  {
    echo ""
    echo "# ---- ROS2 environment (added by container-entrypoint) ----"
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash || true"
    echo "if [ -f \"${ROS2_WS}/install/setup.bash\" ]; then"
    echo "  source \"${ROS2_WS}/install/setup.bash\" || true"
    echo "fi"
    echo "source ${REALSENSE_ENV_FILE} || true"
  } >> "${BASHRC}"
  log "Appended ROS2 + RealSense sourcing to ${BASHRC}"
fi

# Give final status (quick checks)
log "Quick checks:"
log "  - LIBREALSENSE_USB_BACKEND=${LIBREALSENSE_USB_BACKEND}"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  if rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
    log "  - rs-enumerate-devices: RealSense device(s) detected"
  else
    log "  - rs-enumerate-devices: no device detected (plug camera & retry)"
  fi
else
  log "  - rs-enumerate-devices: not installed in container"
fi
echo ""

log "You are now root inside the container. Useful commands:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${ROS2_WS}/install/setup.bash"
echo "  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual model:=m1013"
echo ""

# Exec a login shell so /etc/profile.d and /root/.bashrc are applied
exec /bin/bash --login
