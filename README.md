## Description
The simulation is based on the Open Construction Simulator (OCS). Open Construction Simulator (OCS) is a free construction simulator.  
It is developed based on the game engine "Unity" and provides a simulation environment for excavation and earth transportation using heavy machinery.  

The link to the repo:  
https://github.com/Field-Robotics-Japan/OpenConstructionSimulator 

The implementations of the ROS2 sensors in unity in this repository are based on the following repositories:  
https://github.com/Field-Robotics-Japan/OpenConstructionSimulator  
https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example  
https://github.com/lgsvl/simulator/tree/release-2020.05  
Please check them if you have more interests.  


## Environment
### Unity Version
2020.2 or later

### Depend Packages
Burst : >=1.4.8  
Mathematics : >=1.2.1  
ROS TCP Connector : >=v0.5.0  
UnitySensors : >=0.1.0

## Installation
Open the package manager from `Window -> Package Manager` and select "Add package from git URL..."
Enter the following each URLs, respectively.  

For ROS connection:  
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

For urdf importer:  
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer  

For sensors used in the simulation:  
https://github.com/Field-Robotics-Japan/UnitySensors.git  
https://github.com/Field-Robotics-Japan/UnitySensorsROS.git#v0.1.0

Click Install buton on the right bottom corner for each depend packages, respectively.

## Other Requirements
(optional)The GPS sensor depends on nmea_msgs, if you want to use GPS sensor and it is not installed beforehand, please install this:  

`$ sudo apt-get install ros-<ros-distro>-nmea-msgs`

## Quick Start

### 1. Open project
Finally, please open `OpenConstructionSimulator` package from UnityHub. (It takes more than 5 minuites at the first time, in the case).

### 2. Select the Scene file
There are pre-built Scene file in `Asset/OpenConstructionSim/Scenes/AdaptedDumperEnv.unity`. Check the sensor scripts and the controller scripts under 'dumper' object if you have more interests to adapt the scripts to new models.


### 3. Connect with ROS
please refer to the Unity-Robotics-Hub for ROS connection:
https://github.com/Unity-Technologies/Unity-Robotics-Hub









