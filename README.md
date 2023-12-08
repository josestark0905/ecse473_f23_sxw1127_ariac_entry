# ecse473_f23_sxw1127_ariac_entry
## Prepare for the environment (ROS Noetic)
1. Create a catkin workspace and build the simulation environment
```
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git
rosdep install --from-paths cwru_ariac_2019 --ignore-src -r -y
cd ../
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"
```


2. Create another catkin workspace for ecse_373_ariac
```
mkdir -p ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y
cd ../
catkin_make
```

3. Some other tips
1) `source /opt/ros/noetic/setup.bash` can be used if your system cannot automatically be a ware of the new installed packs.
2) The ecse_373_ariac can also be installed in the root environment of ROS with `sudo -- /bin/bash -c` method similar to cwru_ariac_2019. But I don’t recommend this, because after I tried to do this, the permissions of ecse_373_ariac were changed to root permissions, which caused compilation problems.


## Build
1. `cd <workspace>`
2. `catkin_make`
## Setup the environment
`source devel/setup.bash`
## Launch the competition environment
`roslaunch ecse_373_ariac ecse_373_ariac.launch`
## Start the client
`rosrun ariac_entry ariac_entry_node`
