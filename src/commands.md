# Useful Commands

Build **ros2** package(only needed if added or removed a `.urdf` file):<br>
    `colcon build --symlink-install`

Launch **ros2** package:<br>
    `ros2 launch terrence_2 rsp.launch.py`

Launch **ros2** package with *simulation* (Note: `world:=` is optional):<br>
    `ros2 launch terrence_2 rsp_sim.launch.py world:=src/worlds/[world_name].world`

Run **rviz2** with a config file:<br>
    `rviz2 -d src/config/[config_name].rviz`

Run the **Joint State Publisher GUI**:<br>
    `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

Run **Gazebo**:<br>
    `ros2 launch gazebo_ros gazebo.launch.py`

Spawn a `.urdf` into **Gazebo**:<br>
    `ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity terrence_2`