# robot_ur3e_manipulation

- [robot\_ur3e\_manipulation](#robot_ur3e_manipulation)
  - [Service version](#service-version)
  - [Action version](#action-version)
  - [Advanced version](#advanced-version)
  - [Primitive version](#primitive-version)

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