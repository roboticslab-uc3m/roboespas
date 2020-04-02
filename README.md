- Download the whole repo wherever you want to have your workspace, for example, in the ```/home/user``` folder

```
cd ~
git clone https://github.com/Leytha/roboespas_ws
```

- Modify .bashrc to use this workspace and set the ROS_IP and ROS_WORKSPACE env variables. Change the IP for your IP (check it using ```ifconfig```).
```
export ROS_WORKSPACE=~/roboespas_ws
export ROS_MASTER_URI=http://192.168.1.53:11311
export ROS_IP=192.168.1.53
```
Restart the terminal and check it worked by typing 
```
roscd
```
It should lead you to ```~/roboespas_ws```
- Compile the workspace
```
roscd
cd src/
catkin_make
```

- Execute in an Ubuntu terminal: ```roslaunch iiwa_gazebo iiwa_world.launch```.

- In Matlab, modify the IP in the file ```LBRTorqueControlExample.m``` (lines 30-31) inside the ```MatlabExamples``` folder, then play the script and watch IIWA in Gazebo simulator moving.
