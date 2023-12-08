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
2) The ecse_373_ariac can also be installed in the root environment of ROS with `sudo -- /bin/bash -c` method similar to cwru_ariac_2019. While I don’t recommend this, since after I tried to do this, the permissions of ecse_373_ariac were changed to root mode, which caused compilation problems.


## Build
1. Download this repository to your `<path to your workspace>/src`.
2. `cd <path to your workspace>`
3. `source <path to ecse_373_ariac workspace>/devel/setup.bash`
4. `catkin_make`
5. `source <path to your workspace>/devel/setup.bash`


## Launch the competition environment
`roslaunch ecse_373_ariac ecse_373_ariac.launch`


## Start the competition
`rosrun ariac_entry ariac_entry_node`


## Result of the competition
1. The highest final score is 19
![Final_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/055f3feb-ae4d-4030-9e91-6fabaf3bd7c6)
![order_0_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/bb4d0775-38f0-4f3b-af0e-354ad3591bc6)
![order_1_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/53eb4fcf-e6fe-4e70-a8be-551d98a10fc0)
2. It's not stable, there are some unknown factors may impact the performance of the arm. For example, the inverse function in "ur_kinematics" sometimes gives 0 possible solutions, even if the arm is near the bin. So I tried to solve this problem. I checked the code in `ecse_373_ariac/ur_kinematics/src/ur_kin.cpp` and found that the "inverse" function is just a calculation based on the formula, which will not lead to 0 solutions itself. Therefore, it can be inferred that the unstable factor is the "lookuptransform" function.
3. I adjusted the parameters to strength the stability, now it seldom get 0 solutions. When the arm suddenly move in an unexpected way, ROS_INFO "GOT THE TRAJECTORY 0" can be seen in the terminal, which means there are no possible solutions. In this condition,Just shut down both the ariac_entry_node and the ecse_373_ariac_node and rerun them, then it will work.
