- cd
- git clone https://github.com/phattanaratjeedjeen-sudo/solve_maze.git
- cd ~/solve_maze
- colcon build && source install setup.bash

at PC station
- ros2 launch scantest all.launch.py
- ros2 launch scantest standalone.launch.xml

set target grid
- ros2 param set /state_manager_node target_x <int>
- ros2 param set /state_manager_node target_x <int>

set initial grid
- ros2 param set /pose_converter_node initial_grid_x <float>
- ros2 param set /pose_converter_node initial_grid_y <float>

set initial grid

at PI5
- ros2 launch scantest laser_active.py
