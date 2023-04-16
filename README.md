## Description
The simulation is based on the Open Construction Simulator (OCS).  

The repo link:  
https://github.com/Field-Robotics-Japan/OpenConstructionSimulator 

Open Construction Simulator (OCS) is a free construction simulator.  
It is developed based on the game engine "Unity" and provides a simulation environment for excavation and earth transportation using heavy machinery.

**note**  
The version maintained in this repository is a **Demo** version.  
[The Full version](https://github.com/qoopen0815/OpenConstructionSimulator/releases) is distributed only as a binary due to license restrictions.

![222_Trim](https://user-images.githubusercontent.com/26988372/133398942-6b8ef0e1-ac1b-4119-a4f6-ea16bbeaaa40.gif)

In this repository, following packages are utilized.
Please check them if you have more interests.

- [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors) : Sensor packages available for Unity
- [OcsSystem](https://github.com/qoopen0815/OcsSystem) : System management package
- [OcsTerrain](https://github.com/qoopen0815/OcsTerrain) : Terrain control package
- [OcsVehicle](https://github.com/qoopen0815/OcsVehicle) : Vehicle control package

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
The GPS sensor depends on nmea_msgs, if not installed beforehand, please install this:  

`$ sudo apt-get install ros-<ros-distro>-nmea-msgs`

## Quick Start

### 1. Open project
Finally, please open `OpenConstructionSimulator` package from UnityHub. (It takes more than 5 minuites at the first time, in the case)

### 2. Select the Scene file
There are pre-built Scene file in `Asset/OpenConstructionSim/Scenes/NewEnv.unity`.  


### 3. Connect with ROS
please refer to the Unity-Robotics-Hub for ROS connection:
https://github.com/Unity-Technologies/Unity-Robotics-Hub









