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
`mkdir -p ~/ecse_373_ariac_ws/src`
`cd ~/ecse_373_ariac_ws/src`
`git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git`
`rosdep install --from-paths ecse_373_ariac --ignore-src -r -y`
`cd ../`
`catkin_make`



## Build
1. `cd <workspace>`
2. `catkin_make`
## Setup the environment
`source devel/setup.bash`
## Launch the competition environment
`roslaunch ecse_373_ariac ecse_373_ariac.launch`
## Start the client
`rosrun ariac_entry ariac_entry_node`
