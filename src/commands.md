# Useful Commands

### Terminal 1:

Build **colcon** package(only needed if added or removed a `.urdf` file):<br>
    `colcon build --symlink-install`

Launch **colcon** package:<br>
    `ros2 launch terrence_2 rsp.launch.py`

### Terminal 2:

Run **rviz2** with a config file:<br>
    `rviz2 -d src/config/view_bot.rviz`

### Terminal 3:

Run the **Joint State Publisher GUI**:<br>
    `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
