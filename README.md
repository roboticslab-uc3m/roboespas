- If you haven't created a catkin_workspace, do so
cd ~
mkdir your_ws
cd your_ws
mkdir src

- Download the packages in the src folder
```
cd ~
git clone https://github.com/roboticslab-uc3m/roboespas
```

- Modify .bashrc to use this workspace and set the ROS_IP and ROS_WORKSPACE env variables. Change the IP for your IP (check it using ```ifconfig```).
```
export ROS_WORKSPACE=~/your_ws    #edit for your workspace
export ROS_MASTER_URI=http://192.168.1.53:11311   #edit for your IP
export ROS_IP=192.168.1.53  #edit for your IP
```
- Compile the workspace
```
roscd
cd src/
catkin_make
```
## Torque command example
### Option 1: Control iiwa_gazebo without using iiwa_command action server
This way Matlab will send joint torque values point by point.
- Execute in a terminal: ```roslaunch iiwa_command iiwa_command_gazebo.launch```. Although you won't use iiwa_command action server, you need to use the launcher to load config parameters.

- In Matlab, modify the IP in the file ```TorqueControlExample.m``` (lines 6-7) inside the ```TorqueControlExample``` folder, then play the script and watch IIWA in Gazebo simulator moving.

### Option 2:Control iiwa_gazebo using iiwa_command action lib
This way Matlab will send joint torque values all together as a trajectory.
- Execute in a terminal: ```roslaunch iiwa_command iiwa_command_gazebo.launch```.

- In Matlab, modify the IP in the file ```TrajectoryTorqueControlExample.m``` (lines 6-7) inside the ```TorqueControlExample``` folder, then play the script and watch IIWA in Gazebo simulator moving.
