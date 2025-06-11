all:
	colcon build --symlink-install --packages-select my_create my_create_description my_create_bringup my_create_driver


# Do this to run my_create_launch.py
# ``` python
# source install/setup.bash && make run
# ```
# And add venv package to path
# ``` python
# export PYTHONPATH=/home/jordenhuang/webots_ros2/.venv/lib/python3.10/site-packages:$PYTHONPATH

run:
	ros2 launch my_create_bringup my_create_launch.py
