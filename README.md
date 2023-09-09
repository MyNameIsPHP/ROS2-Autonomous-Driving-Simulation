# ROS2 Autonomous Driving Simulation

## Prerequisites
* Python 3.8
* Platforms: Ubuntu 20.04, CUDA Version: 12.2
* ROS2 Foxy, Gazebo 11 
* pip install: opencv-python, scikit-fuzzy, onnxruntime

## Usage
* Clone and get into this repository
```
cd 
git clone https://github.com/MyNameIsPHP/ROS2-Autonomous-Driving-Simulation.git
cd ROS2-Autonomous-Driving-Simulation/
```

* Copy all models in **src/prius_sdc_pkg/model/** into your **~/.gazebo/models**

* Perform Colcon Build
```
source /opt/ros/foxy/setup.bash 
colcon build
```
* Source your Workspace 
```
source install/setup.bash 
```
* Launch world
 ```
  ros2 launch prius_sdc_pkg world.launch.py 
 ```
* Run driver node
 ```
  cd src/prius_sdc_pkg/prius_sdc_pkg/
  ros2 run prius_sdc_pkg driver_node  
 ```

