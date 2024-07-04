# robot_ur3e_manipulation

```
git clone https://github.com/Andy-Leo10/robot_ur3e_manipulation.git
```

- [robot\_ur3e\_manipulation](#robot_ur3e_manipulation)
- [SIMULATED ROBOT](#simulated-robot)
  - [Service version](#service-version)
  - [Action version](#action-version)
  - [Advanced version](#advanced-version)
  - [Primitive version](#primitive-version)
- [REAL ROBOT](#real-robot)
  - [Service version](#service-version-1)
  - [Action version](#action-version-1)
  - [Advanced version](#advanced-version-1)

---

<details>
<summary><b>SIMULATED ROBOT</b></summary>

# SIMULATED ROBOT
## Service version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation sim_service_server.launch.py
```
Calling to service to order a coffee
```
ros2 service call /robot_ur3e_manipulation_ss robot_ur3e_manipulation/srv/DeliverCoffeeService 'coffe_order: true'
```

## Action version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation sim_action_server.launch.py
```
Calling to action to order a coffee
```
ros2 action send_goal /robot_ur3e_manipulation_as robot_ur3e_manipulation/action/DeliverCoffeeAction 'coffe_order: true' -f
```

## Advanced version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation sim_pick_and_place_advanced.launch.py
```

## Primitive version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation sim_pick_and_place_primitive.launch.py
```
</details>

---

<details>
<summary><b>REAL ROBOT</b></summary>

# REAL ROBOT
## Service version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation real_service_server.launch.py
```
Calling to service to order a coffee
```
ros2 service call /robot_ur3e_manipulation_ss robot_ur3e_manipulation/srv/DeliverCoffeeService 'coffe_order: true'
```

## Action version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation real_action_server.launch.py
```
Calling to action to order a coffee
```
ros2 action send_goal /robot_ur3e_manipulation_as robot_ur3e_manipulation/action/DeliverCoffeeAction 'coffe_order: true' -f
```

## Advanced version
```
cd ~/ros2_ws/; colcon build --packages-select robot_ur3e_manipulation; source install/setup.bash; ros2 launch robot_ur3e_manipulation real_pick_and_place_advanced.launch.py
```
</details>

---