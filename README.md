## Requirements
### Ubuntu RT patch
Check your current kernel version
```uname -a```
If it is not a RT kernel version (it does not contain the word "RT"), install the RT patch.
Follow the instructions here: https://stackoverflow.com/questions/51669724/install-rt-linux-patch-for-ubuntu. To choose the RT kernel version, select the next kernel version which has a RT patch here: https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/.
For example, if ```uname -a``` returned *4.15.0-112-generic*, choose RT kernel *4.16.18-rt12*.
### ROS Kinetic
http://wiki.ros.org/kinetic/Installation/Ubuntu
## Configuration
- Download the repo
```
git clone https://github.com/roboticslab-uc3m/roboespas
```
- Change to develop if necessary
```
git checkout develop
```
- Modify .bashrc to use this workspace and set the ROS_IP and ROS_WORKSPACE env variables. Change the IP for your IP (check it using ```ifconfig```).
```
export ROS_WORKSPACE=~/your_ws    #edit for your workspace
export ROS_MASTER_URI=http://192.168.1.53:11311   #edit for your IP
export ROS_IP=192.168.1.53  #edit for your IP
```
- Compile and install FRI-Client-SDK
```
roscd
cd src/iiwa_fri/FRI-Client-SDK_Cpp/
mkdir build
cd build
cmake ..
make -j8
sudo make install
```

- Fix Eigen dependencies

- Compile the ROS workspace
```
roscd
catkin_make
```



- Re-generate messages (jump to 3 if you have not generate matlab messages before)

1. Delete previously created messages.
In Matlab prompt
```
rosgenmsg('/home/user/your_ws/src/roboespas')
```
Open ```javaclasspath.txt``` file that you are linked and delete all the content.
Delete ```/home/user/your_ws/src/roboespas/matlab_gen``` folder.

2. Restart Matlab

3. Generate messages
```
rosgenmsg('/home/user/your_ws/src/roboespas')
```
Follow the instructions (add to javaclasspath.txt the files given, and add to path the folder)
4. Restart Matlab.

## Torque command example
### Option 1: Control iiwa_gazebo without using iiwa_command action server
This way Matlab will send joint torque values point by point.
- Execute in a terminal: ```roslaunch iiwa_command iiwa_command_gazebo.launch```. Although you won't use iiwa_command action server, you need to use the launcher to load config parameters.

- In Matlab, modify the IP in the file ```TorqueControlExample.m``` (lines 6-7) inside the ```TorqueControlExample``` folder, then play the script and watch IIWA in Gazebo simulator moving.

### Option 2:Control iiwa_gazebo using iiwa_command action lib
This way Matlab will send joint torque values all together as a trajectory.
- Execute in a terminal: ```roslaunch iiwa_command iiwa_command_gazebo.launch```.

- In Matlab, modify the IP in the file ```TrajectoryTorqueControlExample.m``` (lines 6-7) inside the ```TorqueControlExample``` folder, then play the script and watch IIWA in Gazebo simulator moving.


## Other files used
### Dataset trajectories:
https://drive.google.com/drive/folders/1mvBHLigwaf9ykC70j0tyhNbo1mdIiZAx

### Extreme Learning Machine
https://es.mathworks.com/matlabcentral/fileexchange/69812-extreme-learning-machine-for-classification-and-regression
