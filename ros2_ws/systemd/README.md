# Autostart on the Pi (systemd)

Power the car on and it is ready for the start button — no laptop needed.

| Unit | Runs | Launch file |
| --- | --- | --- |
| `gorur-base.service` | `mcu_bridge`, `vector_odom`, RPLIDAR C1 (`sllidar_ros2`, built in `~/lidar_ws`) | `launch/gorur_base.launch.py` |
| `gorur-round.service` | `local_comp` `open_round` **or** `obstacle_round`, per `config/round_select.yaml` | `launch/gorur_round.launch.py` |

Both round nodes wait in STANDBY until the ESP32's BOOT button (`/button_status`) is pressed,
then drive exactly `start_delay_sec` (5 s) later while the status LED blinks the delay out once
a second and goes solid as the car moves. Only one round runs at a time (both own `/cmd_vel`); `gorur-round` is restarted together with
`gorur-base` (`PartOf=`). Every node respawns if it dies, so the MCU and LiDAR can be plugged in
after boot.

## Install (once per Pi, re-run after pulling)

```bash
cd ~/gorur_gari_2026/ros2_ws
./systemd/install.sh            # deps, udev rules, colcon build, enable + start the units
```

**What the start button starts** lives in [`config/round_select.yaml`](../config/round_select.yaml):

```yaml
round: open            # open -> local_comp open_round, obstacle -> local_comp obstacle_round
start_delay_sec: 5.0   # press -> first drive command; must match the firmware LED countdown
```

Edit it and `sudo systemctl restart gorur-round`, or let `gorur-round open|obstacle` do both.
Everything else lives in `/etc/default/gorur` (workspace path, `ROS_DOMAIN_ID`, extra launch args;
`GORUR_ROUND_ARGS="round:=obstacle"` overrides the file without editing it).
The units run as user `goru` from `/home/goru/gorur_gari_2026/ros2_ws`.

## Day to day

```bash
gorur-round                     # which round, start delay + service status
gorur-round obstacle            # button starts the obstacle round (edits round_select.yaml)
gorur-round open                # back to the open round

journalctl -fu gorur-base -u gorur-round      # live logs
sudo systemctl restart gorur-base             # restart everything
sudo systemctl stop gorur-round               # stop the driver, keep MCU + LiDAR up
sudo systemctl disable gorur-base gorur-round # no autostart (e.g. for manual testing)
```

After `git pull`, Python changes take effect on `sudo systemctl restart gorur-base`
(symlink install); new files, entry points or packages need `colcon build` first
(or just re-run `install.sh`).
