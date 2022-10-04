# ECE598JK Assigment 2 Instructions

Clone MINI_Upgrade_Kit repo from Github

```
mkdir -p ~/ws_mini/src && cd ~/ws_mini/src
git clone -b assn2 https://github.com/uiuckimlab/MINI_Upgrade_Kit.git
```

Make using catkin and source ros workspace env

```
cd ~/ws_mini && catkin_make
```

Terminal A: launch simulator and ros controller nodes

```
cd ~/ws_mini && source devel/setup.bash && roslaunch robotis_mini_control sim.launch
```

Start the simulation by clicking the play button on the botton left of the Gazebo UI.

Terminal B: launch rviz and configs

```
cd ~/ws_mini && source devel/setup.bash && roslaunch robotis_mini_control rviz.launch
```

Terminal C: rosrun assignment script

```
cd ~/ws_mini && source devel/setup.bash && rosrun robotis_mini_control foot_position.py
```

Terminal D: Use rqt_reconfigure tool to control the static foot position
```
cd ~/ws_mini && source devel/setup.bash && rosrun rqt_reconfigure rqt_reconfigure 
```
![alt text](http://url/to/img.png)

# ROS Setup and Tips

Install ROS Noetic

```
sudo apt-get update
sudo apt-get upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
sudo apt update
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Install tools and setup env paths

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

Install Terminator

```
sudo apt install terminator
```

VScode

```
sudo snap install code --classic
code ~/ws_mini/src/MINI_Upgrade_Kit
```

Install dependencies

```
sudo apt-get install ros-noetic-dynamixel-sdk \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-plotjuggler-ros
```

```
sudo apt install python3-pip
pip3 install urdf_parser_py scipy
```

## Analyze data with PlotJuggler

```
cd ~/ws_mini && source devel/setup.bash && rosrun plotjuggler plotjuggler
```

Streaming -> ROS Topic Subscriber -> Add topics of interest

```
robotis_mini/joint_states/l_ankle_joint/position
/l_foot_Fz and /l_foot_Fz
```

right click on plot -> split vertically
-> apply filter to data

## Graph of ros nodes and topics

```
rosrun rqt_graph rqt_graph
```

Hide Debug, Unreachable, and Params

## Control individual joints with ros_control GUI

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

controller manager ns -> /robotis_mini/controller_manager
controller -> full_body_controller
click power button and set speed scaling to 100%

## Setting up Rviz Visualization Config

```
rosrun rviz rviz
```

- Global Fixed Visualization Frame
- RobotModel
- TF
- Marker
- Camera

Place object in from of the mini in Gazebo World

## Record and play back rosbag

```
rosbag record -O robotis_mini_demo.bag l_foot_Fz r_foot_Fz robotis_mini/joint_states
rosbag info robotis_mini_demo.bag
rosbag info -y -k duration robotis_mini_demo.bag
rosbag play robotis_mini_demo.bag
rostopic echo -b robotis_mini_demo.bag -p /joint_states robotis_mini_demo.csv
```

## Exploring RQT tools

```
rqt
```

- Topics
  - Message publisher
  - Topic Monitor
- Visualization
  - Plot
  - TF Tree
  - Image View
- Other
  - Bag
  - rqt_joint_trajectory_controller
  - Dynamic reconfigure
  - Process monitor
  - Message Type Browser
