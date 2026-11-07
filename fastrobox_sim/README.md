# fastrobox_sim

Simulation of the **BUBT FastRobox 1.0 Smart Navigation Challenge**: a robot with one camera drives a 3 m track, going left of red boxes and right of green ones, until it reaches the finish. It runs on ROS 2 Humble with Ignition Gazebo Fortress.

| Deliverable | File |
| :-- | :-- |
| Gazebo world | [`worlds/competition_arena.world`](worlds/competition_arena.world) |
| Robot model | [`description/robot.urdf.xacro`](description/robot.urdf.xacro) |
| Control node | [`fastrobox_sim/smart_navigation.py`](fastrobox_sim/smart_navigation.py) |
| Random box placement | [`fastrobox_sim/spawn_boxes.py`](fastrobox_sim/spawn_boxes.py), [`fastrobox_sim/box_layout.py`](fastrobox_sim/box_layout.py) |
| Launch file | [`launch/competition.launch.py`](launch/competition.launch.py) |

## Running it

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths fastrobox_sim --packages-select fastrobox_sim \
             --build-base ros2_ws/build --install-base ros2_ws/install --symlink-install
source ros2_ws/install/setup.bash

ros2 launch fastrobox_sim competition.launch.py
```

The Gazebo window opens, the boxes appear, and the robot sets off. When it stops, the referee prints its verdict in the terminal.

| Argument | Default | What it does |
| :-- | :-- | :-- |
| `seed` | new layout each run | Replays a layout. Every run prints its seed |
| `box_count` | 6 to 8 at random | 1 to 8 boxes |
| `gui` | `true` | `false` runs headless |
| `drive` | `true` | `false` starts everything except the node and the referee |
| `layout_file` | `/tmp/fastrobox_layout.yaml` | Where the layout is recorded for the referee |
| `result_file` | none | Where the referee writes its verdict as JSON |

To see what the robot sees, with its detections drawn on:

```bash
ros2 run rqt_image_view rqt_image_view /nav/debug_image
```

To drive it yourself, launch with `drive:=false`, then:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## The arena

The track runs along +x. `x = 0` is the back of the start zone, `x = 3` the end of the finish zone, and `y = 0` the centre line, with +y to the robot's left.

| | Specification |
| :-- | :-- |
| Track | 3.0 × 1.0 m, enclosed by walls 0.2 m high |
| Start zone | `x` 0 to 0.2 m, light green |
| Finish zone | `x` 2.8 to 3.0 m, dark green and white 0.1 m checks |
| Boxes | 0.05 × 0.05 × 0.10 m, solid red or green, free to be knocked over |

## The robot

Four-wheel skid-steer, 0.19 m long and 0.15 m wide. The camera is on a mast at the front, 0.16 m up and tilted 20° down, with an 80° horizontal field of view at 640×480.

| Topic | Type | |
| :-- | :-- | :-- |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs` | The camera |
| `/cmd_vel` | `geometry_msgs/Twist` | Drive command |
| `/imu` | `sensor_msgs/Imu` | Gyro, 100 Hz. Heading comes from this |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry. Only its speed is used |
| `/ground_truth` | `nav_msgs/Odometry` | True pose, for the referee only |
| `/nav/state` | `std_msgs/String` | `FORWARD`, `TURN_LEFT`, `TURN_RIGHT` or `FINISHED` |
| `/nav/debug_image` | `sensor_msgs/Image` | Camera frame with detections drawn on |

## How the node drives

1. **See.** It thresholds each frame in HSV and finds where each box's base meets the floor. The camera's height and tilt turn that pixel into a position on the floor.
2. **Remember.** It dead-reckons its pose from gyro heading and wheel speed, and keeps each box's position in that frame, because a box leaves the camera's view while the robot is still alongside it.
3. **Choose a lane.** A red box within 0.7 m requires the robot to be left of it, and a green box right of it. A box already on the correct side needs no turn.
4. **Steer.** It heads for that lane, then straightens up along the track and carries on.
5. **Finish.** It locates the finish line with the camera, and stops with its base centre 7 cm past the line.

## Where this goes beyond the brief

The brief left some things open, or could not work as written. These are the decisions to check against the official rules:

- **The robot has an IMU, and the node also uses it and `/odom`.** The brief lists only the camera. But "turn, then resume moving forward" without knowing how far it turned leaves the robot on the new heading until it hits a side wall. Wheel odometry alone isn't enough, because a skid-steer turns by slipping its wheels: in one measured run it finished 18° and 50 cm from the truth, and 7 of 20 layouts failed. If the rules forbid extra sensors, this needs rethinking.
- **Every box is passed on its colour's side, wherever it stands.** A red box 0.2 m to the robot's left still sends the robot left, around it. If the rules only mean "turn when one is directly in your path", that is less strict, and this still satisfies it.
- **The node is `smart_navigation.py`.** The brief calls it `nav_node.py` in one place and `smart_navigation.py` in the deliverables; the deliverables list won.
- **The robot is 0.19 m long, not 0.2 m**, because the start and finish zones are exactly 0.2 m long. At 0.2 m it could only stand in a zone by touching the end wall. It still doesn't fit in the finish zone with room to spare, so "finished" here means that the base centre stopped inside the zone.
- **The finish zone is dark green and white checks**, not bright green, because bright green would look like a green box. They differ in saturation, which is what the node tests.
- **Layouts guarantee a legal route, not just free space.** A red box against the left wall has room around it, but none on the side the rules require. See [`box_layout.py`](fastrobox_sim/box_layout.py). This keeps boxes at least 0.30 m apart along the track, and a dense layout tends to have red boxes to the right of green ones, because that is what leaves a route.

## Checking it

The referee scores a run from ground truth, which the robot never sees. A run passes if the base stopped inside the finish zone, every box was passed on its legal side, and the footprint touched no box and no side or end wall.

```bash
# Unit tests and lint, no simulator needed. PYTHONNOUSERSITE: see Known issues.
cd fastrobox_sim && PYTHONNOUSERSITE=1 python3 -m pytest test

# The whole challenge, headless, over many random layouts.
python3 tools/run_trials.py --seeds 0-19
```

RESULTS_PLACEHOLDER

## Known issues and limits

- **NumPy 2 in `~/.local` crashes `cv_bridge`.** ROS Humble's `cv_bridge` is built against NumPy 1.x. The launch file sets `PYTHONNOUSERSITE=1` for the node, so it is only a problem if you run the node by hand or run the tests without it.
- **The camera's near clip is 0.1 m.** At 0.02 m, a flat dark band covered the frame between the horizon and the image centre, hiding boxes. Raising the clip removes it; the source of the band was not identified.
- **Rendering is cleaner than a real camera.** Lighting is even, there are no shadows, and colours are constant. The HSV thresholds are measured on these renders and need re-measuring on a real camera.
- **`effective_track` is measured, not modelled.** The value in the xacro holds for these wheels and this friction, and needs re-measuring if either changes.
- **The simulated gyro is optimistic.** It has noise but no bias, and a real one would drift more.
- **The robot must start on the centre line, facing along the track.** It treats its starting pose as the track's axis.
