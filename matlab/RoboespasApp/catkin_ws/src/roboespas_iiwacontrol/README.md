# roboespas_iiwacontrol
## 1. Requirements
### Ubuntu 16.04
- Check https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0
### ROS Kinetic with Gazebo
- Check http://wiki.ros.org/kinetic/Installation/Ubuntu
- Make sure ```control_toolbox```, ```controller_interface```, ```controller_manager``` and ```joint_limits_interface``` are installed
```
sudo apt-get install ros-kinetic-control-toolbox
sudo apt-get install ros-kinetic-controller-interface
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-joint-limits-interface
```
*For any problem with Gazebo, check http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros*

### Eigen3 v3.3.7
- Download and uncompress Eigen v.3.3.7
``` cd ~/Downloads
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
tar xvf 3.3.7.tar.gz
```
- Copy the folder ```Eigen``` into ```/usr/local/include```, you may remove the download if you want to.
```
sudo cp -r eigen-eigen-323c052e1731/Eigen/ /usr/local/include
sudo rm -r eigen-eigen-323c052e1731/
```
### Matlab R2019a
- Download from https://es.mathworks.com/downloads/web_downloads/download_release?release=R2019a
- Follow the installation instructions
### Robotics System Toolbox
This toolbox allows us to connect Matlab with ROS
- Matlab > Add-ons > Robotics System Toolbox > Install
### Robotics System Toolbox for ROS Custom Messages
This toolbox allows us the usage of ROS custom messages such as iiwa_msgs or roboespas_iiwacontrol services from Matlab
- Matlab > Add-ons > Robotics System Toolbox for ROS Custom Messages > Install
## 2. Configure roboespas_iiwacontrol package in your ROS workspace
### Create a new catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
### Download iiwa_stack v1.2.5 (Salvo Virga)
- Download  and uncompress ```iiwa_stack``` v.1.2.5
``` cd ~/Downloads
wget https://github.com/IFL-CAMP/iiwa_stack/archive/1.2.5.tar.gz
tar xvf 1.2.5.tar.gz
```
- Copy all the folders inside the downloaded folder to the ROS workspace source folder, you may remove the download if you want to.
``` 
sudo cp -r iiwa_stack-1.2.5/* ~/catkin_ws/src
sudo rm -r iiwa_stack-1.2.5/
```
### Download roboespas_iiwacontrol and configure tool handle
- Download from git repository
``` 
cd ~/catkin_ws/src
git clone https://github.com/Leytha/roboespas_iiwacontrol.git
```
The catkin_ws now should look as follows:
```
└──  ~/catkin_ws
   └──  src
        ├── iiwa
        ├── iiwa_control
        ├── iiwa_description
        ├── iiwa_gazebo
        ├── iiwa_hw
        ├── iiwa_moveit
        ├── iiwa_msgs
        ├── iiwa_ros
        ├── roboespas_iiwacontrol
        └── CMakeLists.txt
```
- Configure tool handle: Some files provided inside our package roboespas_iiwacontrol should be copied inside other folder from ```iiwa_stack```. All these files are in the folder ```/roboespas_iiwacontrol/tool_configuration```.
```
cd ~/catkin_ws/src/roboespas_iiwacontrol/tool_configuration/
sudo mv iiwa_gazeboManetaFT.launch ~/catkin_ws/src/iiwa_gazebo/launch/
sudo mkdir ~/catkin_ws/src/iiwa_description/meshes/ManetaFT
sudo mkdir ~/catkin_ws/src/iiwa_description/meshes/ManetaFT/visual
sudo mkdir ~/catkin_ws/src/iiwa_description/meshes/ManetaFT/collision
sudo cp ManetaFT.stl ~/catkin_ws/src/iiwa_description/meshes/ManetaFT/collision/
sudo mv ManetaFT.stl ~/catkin_ws/src/iiwa_description/meshes/ManetaFT/visual/
sudo mv iiwa14ManetaFT.urdf.xacro ~/catkin_ws/src/iiwa_description/urdf/
sudo mv iiwa14ManetaFT_upload.launch ~/catkin_ws/src/iiwa_description/launch/
cd ..
sudo rm -r tool_configuration/
```
### Build catkin_workspace
- Build first the ```iiwa_msgs``` package as they are used in other packages
```
cd ~/catkin_ws
catkin_make --pkg iiwa_msgs
```
- Build the rest of the workspace
```
catkin_make
```
### Configure computer-robot connection
- Connect computer with the iiwa cabinet using an ethernet cable
- Set computer's IP. Open "Net conections" window, add a new connection if not already created, select "Cabled" connection, go to "IPv4 settings" and select as method "Manual", then add a new IP, and set the following values:
```
IP=160.69.69.100
Mask=255.255.255.0
Gateway=160.69.69.1
```
- Save changes and exit

### Modify .bashrc
- Open the .bashrc file
``` 
sudo gedit ~/.bashrc
```
- Add robot IP and set ROS_MASTER URI at the end of the .bashrc file.
```
export ROS_IP=160.69.69.100
export ROS_MASTER_URI=http://$ROS_IP:11311
```
- Set ROS_WORKSPACE variable at the end of the .bashrc file. Change user for your user. 
```
export ROS_WORKSPACE=/home/user/catkin_ws
```
- Source .bash files at the end of the .bashrc file
```
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash
```
- Save and exit gedit

### Generate custom messages from ROS definitions in Matlab
- In Ubuntu, open Matlab R2019a as sudo, in Windows just open Matlab normally.
```
sudo matlab
```
- Generate custom messages from ROS definitions typing the following in Matlab prompt.
```
rosgenmsg('~/catkin_ws/src')
```
- Follow the instructions given by Matlab: modify ```javaclasspath.txt``` and add the matlab_gen folder to the Matlab path as indicated. Now, there is a new folder inside the ~/catkin_ws/src/ called ```matlab_gen```.
- Restart Matlab, this time not necessarily as sudo, and type in the Matlab prompt 
```
rosmsg list
```
to check everything worked correctly. There should appear messages inside the packages ```iiwa_msgs``` and ```roboespas_iiwacontrol```. Although there are no custom messages inside ```roboespas_iiwacontrol``` package, Matlab automatically creates messages for the Request and Response of each service of each package. 
### Add roboespas_iiwacontrol/matlab to Matlab path
- In Ubuntu, open Matlab as sudo, in Windows just open Matlab normally.
```
sudo matlab
```
- Add all the folders of the Matlab ROBOESPAS code to Matlab path
- Save path
```
savepath
```
### Change data path
Open the parameters file and change the data path to wherever you want to read/save the captured trajectories. 
```
sudo gedit ~/catkin_ws/src/roboespas_iiwacontrol/config/params.yaml
```
It should look similar to
```
/home/username/catkin_ws/src/roboespas_iiwacontrol/data
```
There is no need to recompile the package after these changes. 
## 3. Launch
### Tool configuration in the Sunrise Workbench part
If the tool "ManetaFT" is not configurated inside the IIWA Cabinet through the Sunrise Workbench program, the execution may error out. 
### Launch rosmaster
- Execute the roboespas_iiwacontrol launcher with Gazebo
```
roslaunch roboespas_iiwacontrol roboespas_gazebo.launch
```
- Execute the roboespas_iiwacontrol launcher without Gazebo but still using the tool handle attached (IMPORTANT: You should have configured it before in your Sunrise Workbench).
```
roslaunch roboespas_iiwacontrol roboespas.launch
```
### Launch ROSSmartServo
- On the SmartPad, click on Applications > ROSSmartServo > Play
* If it is not there, check https://github.com/IFL-CAMP/iiwa_stack/wiki/sunrise_project_setup 

### Launch Matlab GUI
- Download Matlab files from *TODO*, execute "ROBOESPAS.mlapp"
## 4. Usage

## 5. In-depth
### How to rebuild custom ROS messages
