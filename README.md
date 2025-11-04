- cd
- git clone
- cd ~/solve_maze
- colcon build && source install setup.bash

at PC station
- ros2 launch scantest all.launch.py
- ros2 launch scantest standalone.launch.xml

set target grid
- ros2 param set /
- 

set initial grid

at PI5
- ros2 launch scantest laser_active.py
