# ecse473_f23_sxw1127_ariac_entry
## Theory of Operation
The block diagram shows the theory of operation.
![block_diagram](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/8334c95f-3d14-4344-b1da-d7e1e1990a1c)
The main logic of operation:
1. Get the order by subscribing the topic "/ariac/orders".
2. Extract the information from the orders. Each order contains several shipments. For each shipment, it shows the goal agv to submit, the required kits and the desired pose of each kit in kit_tray.
3. Start from finding the required kits. From the topic "/ariac/material_locations", we can easily get where each kit can be found. Since we do not use the kits on the conveyer, so the "belt" location in the result is ignored.
4. After getting the first bin contains certain kit, we find the pose of the first kit from logical camera of that bin. Before telling the arm how to get the kit, the base of the arm, or we say linear actuator, should be firstly moved near the bin. The position of each bin in world frame is found from the gazebo, {"bin4", 0.383}, {"bin5", 1.15}, {"bin6", 1.916}, {"agv1", 2.2}, {"bin3", -0.383}, {"bin2", -1.15}, {"bin1", -1.916}, {"agv2", -2.2}. More precisely, the linear actuator will only move in the y-axis direction in the world frame, so we only care about the y-coordinate of the bins.
5. In order to avoid the arm from colliding with the logic camera when picking up the kit, we need to add an offset to the goal position of the base. In fact, when we want to get many parts in the same bin, we cannot fix the UR10 base to the same position. For example, when I want to take a part located on the right bottom corner of the bin, and the base position is fixed on the left side of the bin, collision between the arm and the logical camera may still occur. Therefore, for the same bin, two offsets need to be determined, and the base can be fixed on both sides of the bin respectively. Based on the y coordinate of the kit captured by the logical camera, which is the position of the kit in the bin, we can decide which offset to use.
6. After the linear actuator is moved near the bin, we can start to take the kit. The pose of the kit from logical camera is firstly transformed into a pose from the world frame. Then, the kinetic inverse is used to calculate the angel of each joint of UR10 to move the gripper to the transformed pose in the world frame.
7. We first let the gripper stay above the kit, and turn on the vacuum. Then we lower the gripper so that it fully contacts the kit to grab it. The vaccum gripper sometimes does not work, for some unknown reasons. Just repeat this step can make it work. Therefore, we subscribe the topic "/ariac/arm1/gripper/state" that shows whether the kit is successfully grabbed. If not, this step will be repeated again and again.
8. Finally, we place the kit on certain kit_tray extracted from the order. The procedure of moving the arm is similar to getting the kit.
9. Repeat 3-8 until the required kits in the shipment is all taken to the kit_tray. Submit the shipment and start another shipment, until all shipment in the order is successfully submitted.


## Dependency
The following dependencies should be satisfied:
- Ubuntu 20.04
- ROS(Noetic)
- ARIAC 2019
- ecse_373_ariac (ARIAC expansino for Noetic)


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
1) `source /opt/ros/noetic/setup.bash` can be used if your system cannot automatically be aware of the new installed packs.
2) The ecse_373_ariac can also be installed in the root environment of ROS with `sudo -- /bin/bash -c` method similar to cwru_ariac_2019. While I don’t recommend this, since after I tried to do this, the permissions of ecse_373_ariac were changed to root mode, which caused compilation problems.


## Build
1. Download this repository to your `<path to your workspace>/src`.
2. `cd <path to your workspace>`
3. `source <path to ecse_373_ariac workspace>/devel/setup.bash`
4. `catkin_make`
5. `source <path to your workspace>/devel/setup.bash`


## Launch the competition environment
1. The ariac environment can be started by using `roslaunch ecse_373_ariac ecse_373_ariac.launch` (use the default version, which equals to "python:=true")
2. Don't try to use python:=false, since the range of wrist_2_joint in python verison and non-python version are different. The ariac_entry_node is designed based on the python version ariac environment, so default angle of wrist2 is set to 3.14, which will not work in non-python version

## Start the competition
1. `rosrun ariac_entry ariac_entry_node` can start the ariac_entry node and the node will start the competition and try to finish the competition.
2. It takes time for the ariac environment to be prepared for connection. So if it shows failed to connect server, just wait a few seconds and try again.
3. The lookuptransform function sometimes does't work, which leads to the 0 possible solutions, and it will lead to an error exit. When this occurs, just turn off the environment and rerun it and the node.


## Result of the competition
1. The highest final score is 19
![Final_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/055f3feb-ae4d-4030-9e91-6fabaf3bd7c6)
![order_0_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/bb4d0775-38f0-4f3b-af0e-354ad3591bc6)
![order_1_score](https://github.com/josestark0905/ecse473_f23_sxw1127_ariac_entry/assets/143913141/53eb4fcf-e6fe-4e70-a8be-551d98a10fc0)
2. It's not stable, there are some unknown factors may impact the performance of the arm. For example, the inverse function in "ur_kinematics" sometimes gives 0 possible solutions, even if the arm is near the bin. So I tried to solve this problem. I checked the code in `ecse_373_ariac/ur_kinematics/src/ur_kin.cpp` and found that the "inverse" function is just a calculation based on the formula, which will not lead to 0 solutions itself. Therefore, it can be inferred that the unstable factor is the "lookuptransform" function.
3. I adjusted the parameters to strength the stability, now it seldom get 0 solutions. When the arm suddenly move in an unexpected way, ROS_INFO "GOT THE TRAJECTORY 0" can be seen in the terminal, which means there are no possible solutions. In this condition,Just shut down both the ariac_entry_node and the ecse_373_ariac_node and rerun them, then it will work.
