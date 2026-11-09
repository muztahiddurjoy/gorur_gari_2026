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

```bash
# Inside docker container (Terminal 1)
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch gorusim obstacle_bubt_sim.launch.py
```

## Obstacle Round Node

```bash
# Inside docker container (Terminal 2)
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 run local_comp obstacle_round
```

## Push button command

```bash
# Inside docker container (Terminal 3)
source /opt/ros/humble/setup.bash
ros2 service call /gorusim/press_start std_srvs/srv/Trigger
```

## Reset / Replay from Start

```bash
# Teleport car back to the start line without closing Gazebo (Terminal 3)
ign service -s /world/obstacle_bubt/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req "name: 'goruBot', position: {x: 0.1795, y: 0.0, z: 0.002}, orientation: {x: 0, y: 0, z: 0, w: 1}"
```
