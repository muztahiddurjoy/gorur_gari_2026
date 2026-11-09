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

Launch any of the 10 difficulty tracks directly via `track:=<level>`:

```bash
# Inside docker container (Terminal 1)
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash

# Run default track
ros2 launch gorusim obstacle_bubt_sim.launch.py

# Tier 1: Fundamentals & Progressive Spacing
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level1   # Gentle Offset (Warmup)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level2   # Wide Slalom (Wall Repulsion)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level3   # S-Curve Chicanes (Fast Reversals)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level4   # Asymmetric Bottlenecks (Corridor Squeeze)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level5   # Nightmare Gauntlet (Nose Blocker + Pinch)

# Tier 2: Advanced Stress Tests & Extreme Edge Cases
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level6   # The Funnel (Double Slit + Center Fork)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level7   # Blind Cascade (Staircase Occlusion)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level8   # Wall Squeeze (18cm Razor Margin)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level9   # Asymmetric Cluster (False Dead-End)
ros2 launch gorusim obstacle_bubt_sim.launch.py track:=level10  # Pinball Chaos (Ultimate Edge Cases)
```

### Track Difficulty Overview
- **`level1` (Gentle Offset)**: $dx \ge 35\text{ cm}, y = \pm 0.12\text{ m}$. Broad lanes on both sides.
- **`level2` (Wide Slalom)**: $y = \pm 0.25\text{ m}$ to $\pm 0.28\text{ m}$. Wall repulsion and boundary clearance.
- **`level3` (S-Curve Chicanes)**: $dx = 22\text{ cm}$ to $25\text{ cm}$ rapid direction reversals. Servo slew rate test.
- **`level4` (Bottlenecks)**: Twin obstacles at nearly identical $X$ closing off one side. Disparity gap rejection.
- **`level5` (Nightmare)**: Nose blocker at launch ($x = 0.62\text{ m}$), $18\text{ cm}$ wall pinch, pre-finish gate.
- **`level6` (The Funnel)**: Double-slit twin towers at $x = 0.80\text{ m}$ followed immediately by a dead-center fork at $x = 1.30\text{ m}$.
- **`level7` (Blind Cascade)**: Diagonal staircase barricade from $y = -0.28\text{ m}$ to $y = +0.28\text{ m}$. Pillars occlude each other from the start line.
- **`level8` (Wall Squeeze)**: All pillars hugging side walls at $y = \pm 0.32\text{ m}$ ($18\text{ cm}$ margin to walls). Tests whether side-wall repulsion overshoots.
- **`level9` (Cluster / False Dead-End)**: 3-pillar cluster packed irregularly in the center zone ($x \in [1.15, 1.65\text{ m}]$), looking impenetrable from afar.
- **`level10` (Pinball Chaos)**: Random asymmetric extremes: nose blocker at $x = 0.60\text{ m}$, alternating $18\text{ cm}$ pinches, center traps, and a late swerve right onto the finish threshold ($x = 2.45\text{ m}$).

---

## Obstacle Round Node

Run standalone (using default parameters):
```bash
# Inside docker container (Terminal 2)
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 run local_comp obstacle_round
```

OR run with the dedicated YAML configuration file:
```bash
ros2 run local_comp obstacle_round --ros-args --params-file /workspace/ros2_ws/local_comp/config/obstacle_round.yaml
```

*(For Open Round, use: `ros2 run local_comp open_round --ros-args --params-file /workspace/ros2_ws/local_comp/config/open_round.yaml`)*

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
