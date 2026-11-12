#!/bin/bash
# One-shot Pi setup: dependencies, udev names, workspace build, autostart.
# Re-run it any time (it is idempotent); it rebuilds and restarts the services.
#
#   cd ~/gorur_gari_2026/ros2_ws && ./systemd/install.sh            # set up + enable autostart
#   ./systemd/install.sh --no-enable                                # set up, do not start at boot
set -euo pipefail

SYSTEMD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$SYSTEMD_DIR")"
ENABLE=1
[ "${1:-}" = "--no-enable" ] && ENABLE=0

echo "==> apt dependencies"
sudo apt-get update -q
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
    python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-serial python3-numpy python3-opencv v4l-utils \
    ros-humble-rplidar-ros ros-humble-cv-bridge ros-humble-tf2-ros \
    ros-humble-nav-msgs ros-humble-visualization-msgs ros-humble-launch-ros \
    ros-humble-ament-flake8 ros-humble-ament-pep257 ros-humble-ament-copyright

echo "==> rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update -q || true
set +u; source /opt/ros/humble/setup.bash; set -u
rosdep install --from-paths "$WS" --ignore-src -r -y || true

echo "==> udev rules (/dev/gorur_mcu, /dev/rplidar)"
sudo install -m 644 "$WS/config/99-gorur.rules" /etc/udev/rules.d/99-gorur.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,video "$(id -un)"

echo "==> colcon build"
cd "$WS"
colcon build --symlink-install --packages-select controls autonomy sensors_processing local_comp

echo "==> systemd units"
sudo install -m 755 "$SYSTEMD_DIR/gorur-run" /usr/local/bin/gorur-run
sudo install -m 755 "$SYSTEMD_DIR/gorur-round" /usr/local/bin/gorur-round
if [ ! -f /etc/default/gorur ]; then
    sudo install -m 644 "$SYSTEMD_DIR/gorur.default" /etc/default/gorur
fi
sudo sed -i "s|^GORUR_WS=.*|GORUR_WS=$WS|" /etc/default/gorur
sudo install -m 644 "$SYSTEMD_DIR/gorur-base.service" /etc/systemd/system/
sudo install -m 644 "$SYSTEMD_DIR/gorur-round.service" /etc/systemd/system/
sudo systemctl daemon-reload
if [ "$ENABLE" = 1 ]; then
    sudo systemctl enable gorur-base.service gorur-round.service
    sudo systemctl restart gorur-base.service gorur-round.service
fi
systemctl --no-pager --lines=0 status gorur-base.service gorur-round.service || true
echo "==> done. Round: $(grep ^GORUR_ROUND= /etc/default/gorur). Logs: journalctl -fu gorur-base -u gorur-round"
