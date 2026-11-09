## Docker (Start container & grant GUI display access)

```bash
# Allow local container connections to the X server (host terminal)
xhost +local:root

# Start the docker container in background
docker compose -f docker/compose.yaml up -d

# Open a shell inside the container
docker exec -it gorur_gari_humble bash
```

## Visualizer (Simulator + Gazebo GUI + RViz)

You can launch the default track or any of the 5 difficulty preset tracks using `track:=<level>`:

```bash
# Inside docker container (Terminal 1)
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash

# Run default track
ros2 launch gorusim obstacle_bubt_sim.launch.py

# OR select a difficulty level:
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level1
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level2
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level3
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level4
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level5
```

### Track Difficulty Presets
- **`level1` (Gentle Offset)**: Spaced out towers ($dx \ge 35\text{ cm}$, small offset $y = \pm 0.12\text{ m}$), wide passing corridors on both sides.
- **`level2` (Wide Slalom)**: High lateral displacement ($y = \pm 0.25\text{ m}$ to $\pm 0.28\text{ m}$). Tests side-wall repulsion and driving close to boundaries.
- **`level3` (S-Curve Chicanes)**: Short longitudinal distances ($dx = 22\text{ cm}$ to $25\text{ cm}$) with alternating signs. Tests fast Ackermann steering transitions and latency.
- **`level4` (Bottlenecks)**: Paired obstacles placed at nearly the same $X$ coordinate to narrow the corridor. Tests if the disparity extender properly erases unpassable gaps.
- **`level5` (Nightmare)**: Immediate obstacle dead-center off the start line ($x = 0.62\text{ m}, y = 0.00\text{ m}$), near-wall pinch ($18\text{ cm}$ wall gap), asymmetric chicanes, and pre-finish gate ($x = 2.40\text{ m}$).

---

## Obstacle Round Node

```bash
# Inside docker container (Terminal 2)
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 run local_comp obstacle_round
```

---

## Push Button Command

```bash
# Inside docker container (Terminal 3)
source /opt/ros/humble/setup.bash
ros2 service call /gorusim/press_start std_srvs/srv/Trigger
```

---

## Rebuild & Fast Reset (Without restarting Gazebo)

```bash
# In Terminal 2: Stop node (Ctrl+C), rebuild and restart:
cd /workspace && colcon build --packages-select local_comp && source install/setup.bash
ros2 run local_comp obstacle_round

# In Terminal 3: Teleport car back to the start line:
ign service -s /world/obstacle_bubt/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req "name: 'goruBot', position: {x: 0.1795, y: 0.0, z: 0.002}, orientation: {x: 0, y: 0, z: 0, w: 1}"

# Press start again:
ros2 service call /gorusim/press_start std_srvs/srv/Trigger
```
