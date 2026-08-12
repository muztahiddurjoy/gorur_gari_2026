# lazysim

Simulation of the Gorur Gari 2026 car, on **Ignition Gazebo Fortress**.

The point of this package is that nothing downstream of it knows it is a
simulation. `lazybridge` presents the exact ROS interface
`controls/mcu_bridge` does, and `/scan` looks like the RPLIDAR C1's, so
`vector_odom`, `lap_counter`, `open_round_run`, `vision_node` and
`disparity_extender` all run their real code with no sim-specific
branches.

```
ign gazebo ─┬─ gpu_lidar ──┐
            ├─ imu         ├─ ros_gz_bridge ─┬─▶ /scan  /imu  /odom
            ├─ camera ─────┘                 ├─▶ /camera/image_raw
            └─ joint controllers ◀───────────┘   /lazybot/joint_states
                                                        │
                  /cmd_vel ──▶ lazybridge ◀─────────────┘
                                   │
                                   └──▶ encoder/count, encoder/speed,
                                        encoder/direction, heading,
                                        steering_angle, /button_status,
                                        /joint_states
```

## Running it

```bash
cd <repo>
colcon build --base-paths lazysim ros2_ws \
             --build-base ros2_ws/build --install-base ros2_ws/install \
             --symlink-install
source ros2_ws/install/setup.bash

ros2 launch lazysim open_round_sim.launch.py        # three laps, then home
ros2 launch lazysim obstacle_round_sim.launch.py    # with traffic pillars
ros2 launch lazysim sim.launch.py                   # simulator only, drive it yourself
```

Useful arguments (all three launch files):

| argument | default | what it does |
| :-- | :-- | :-- |
| `gui` | `true` | `false` runs the server headless — much faster |
| `rviz` | `true` | open RViz |
| `enable_auto_steering` | `true` | `false` plans without moving the car |
| `require_button_start` | `false` | `true` waits for the start button |
| `track_config` | `config/track.yaml` | pillar / parking / start layout |
| `bot_config` | `config/bot_config_sim.yaml` | tuning for the whole stack |
| `build_track` | `true` | `false` leaves a bare mat |

Driving it by hand:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.4}}'      # angular.z > 0 steers LEFT
```

Pressing the start button (there is no physical one):

```bash
ros2 service call /lazybot/press_start std_srvs/srv/Trigger
```

## What is modelled, and where the numbers come from

Everything shared with the real car is in `ros2_ws/config/bot_config.yaml`;
`config/bot_config_sim.yaml` mirrors it and marks its four deltas `SIM:`.

| | value | source |
| :-- | :-- | :-- |
| length | 210 mm | `bot.length_m`, README chassis table |
| width over tyres | 120 mm | `bot.width_m` |
| wheel diameter | 50 mm | `bot.wheel_diameter_m` |
| **wheelbase** | **140 mm** | **⚠ not recorded anywhere — see below** |
| steering lock | ±60° | `bot.max_steer_deg` |
| encoder | 363 ticks / wheel rev | `bot.encoder_counts_per_rev` |
| LiDAR | RPLIDAR C1: 720 rays, 10 Hz, 0.05–12 m | `rplidar_ros` launch args |
| IMU | BNO055 at 50 Hz | MCU telemetry poll rate |
| camera | 60° HFOV, 640x480 | `vision_params.yaml camera_hfov_deg` |
| mat | 3 m field, 1 m centre block, 1 m corridor | WRO Future Engineers |

### Things worth knowing

**The wheelbase is a guess.** 140 mm is not measured — it is not in
`bot_config.yaml`, not in the README chassis table, and `cad_files/` is
empty. It sets the turning circle, so measure the real axle-to-axle
distance and correct `wheelbase` in `description/lazyBot.xacro` and
`lazybridge.wheelbase_m` in `config/bot_config_sim.yaml` before trusting
any tight-gap result.

**Wheel diameter disagrees across the repo.** `bot_config.yaml` says
50 mm; both `ros2_ws/launch/*.launch.py` still pass `0.065` to
`vector_odom`, and `vector_odom` itself defaults to `0.065`. The sim uses
50 mm throughout. On the real car that 30% gap goes straight into every
distance odometry reports, so the launch defaults are worth a look.

**Steering commands are inner-wheel angles.** `angular.z = ±1.0` puts the
*inner* front wheel at the 60° stop, and `lazybridge` derives the outer
wheel (40.7° at full lock) from the Ackermann geometry. Reading 60° as a
centreline angle instead would demand 74° of the inner wheel, which no
linkage on the car can reach.

**Pillar colours are not decorative.** `vision_params.yaml` thresholds red
at `S = 255` exactly, so the pillars render with zero green and zero blue
in ambient *and* diffuse and a black specular term. Any white highlight
drops saturation below 255 and makes them invisible to `vision_node`.

**Surfaces need `laser_retro`.** `open_round_run.ray_valid()` discards any
ray with intensity ≤ 0.05. Ignition returns zero intensity for a surface
with no `laser_retro`, so a new obstacle without one is invisible to the
navigation code even though its range comes back perfectly.

**Do not set `use_velocity_commands` on the steering.** Fortress's
`JointPositionController` feeds the position error back with the wrong
sign in that mode; the kingpins run away to their limits and sit there
while the command topic still reads 0.0.

## Known environment issue

`vision_node` cannot start on this machine:

```
from cv_bridge import CvBridge  ->  AttributeError: _ARRAY_API not found
```

ROS Humble's `cv_bridge` is built against NumPy 1.x and this machine has
NumPy 2.2.6. It is unrelated to the simulator — a bare
`python3 -c "from cv_bridge import CvBridge"` fails the same way, and it
breaks `vision_node` on the real robot too. Fix with:

```bash
pip3 install "numpy<2"
```

The sim camera itself is verified correct: a red pillar at 0.5 m reads
`HSV(0, 255, 170)` over 8227 px with an aspect ratio of 1.97, and a green
one `HSV(60, 255, 162)` — all inside the real thresholds, with no
cross-talk. `vision_node` will work unchanged once NumPy is sorted.

## Layout

```
config/
  bot_config_sim.yaml   tuning for lazybridge + the whole autonomy stack
  track.yaml            pillar / parking / start layout
  object_template.sdf   traffic pillar, spawned at runtime
  wall_template.sdf     parking barrier, spawned at runtime
  lazySim.rviz
description/
  lazyBot.xacro         every dimension, in one place
  lazyBotCore.xacro     chassis, wheels, Ackermann linkage
  lidar.xacro           RPLIDAR C1
  IMU.xacro             BNO055
  camera.xacro          USB camera on its pan servo
  control.xacro         Ignition joint controllers, odometry, joint states
  gazebo.xacro          friction
launch/
  sim.launch.py                 simulator only
  open_round_sim.launch.py      + vector_odom, lap_counter, open_round_run
  obstacle_round_sim.launch.py  + vision_node, disparity_extender
lazysim/
  lazybridge.py         stands in for mcu_bridge
  track_maker.py        builds the track over Ignition transport
worlds/
  lazyWorld.sdf         the mat
```
