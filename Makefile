all:
	colcon build --symlink-install --packages-select my_create my_create_description my_create_bringup my_create_driver


# Do this to run my_create_launch.py
# source install/setup.bash && make run
run:
	ros2 launch my_create_bringup my_create_launch.py
