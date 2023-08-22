## Description
The simulation is based on the Open Construction Simulator (OCS). Open Construction Simulator (OCS) is a free construction simulator.
It is developed based on the game engine "Unity" and provides a simulation environment for excavation and earth transportation using heavy machinery.  

The link to the repo:  
https://github.com/Field-Robotics-Japan/OpenConstructionSimulator 

The implementations of the ROS2 sensors in unity in this repository are based on the following repositories:  
https://github.com/Field-Robotics-Japan/UnitySensorsROSAssets  
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
Please open `Simulator` package from UnityHub. (It takes more than 5 minuites at the first time, in the case).

### 2. Select the Scene file
There are pre-built Scene file in `Asset/OpenConstructionSim/Scenes/AdaptedDumperEnv.unity`. Check the sensor scripts and the controller scripts under 'dumper' object if you have more interests to adapt the scripts to new models.  
  
Tips for using radar sensor:  
1. Add the objects in the scene, which you want to detect with radars, to a layer named "Obstacles" (or whatever you want, but one should also modify the script for other names).  
2. Tick the box 'is Trigger' of the colliders for both radars and to be detected objects.
<<<<<<< HEAD

=======
>>>>>>> dev_vslam


### 3. Connect with ROS
Please refer to the Unity-Robotics-Hub for ROS connection:
https://github.com/Unity-Technologies/Unity-Robotics-Hub

### 4. Start automation mode
Tick the box `Option/Automation` in the playing UI after starting the simualtion.


## Folder structure
    ├── README.md                         // help documentation
    
    ├── Assets                            // all models and envroments in unity
        
        ├── OcsVehicle/Models             // store the vehicle models
    
            └── dumper.prefab             // the model we use
        
        ├── OcsVehicle/Scripts/Controller // store the controller scripts specially for the dumper
    
            └── DumperController.cs       // the default differential dumper controller we use
    
        └── OpenConstructionSim/Scenes    // store the simulation environment
    
            └── AdaptedDumperEnv.unity    // the simulation environment we use 
    
	├── scripts                           // store all the controller and sensor scripts

		├── AGVController.cs              // alternative script for differntial controller, if you don't want to use the default one

		├── OmniController.cs             // omnidirectional controller script

		├── LaserScanSensor.cs            // 2D LIDAR script we use

		├── RGBCamera.cs                  // RGB camera script

		├── DepthCamera.cs                // depth camera script

		├── RadarMesh.cs                  // RADAR mesh script for generating radar mesh

		├── RadarSensor.cs                // RADAR sensor script

		├── Ultrasonic.cs                 // 3D ultrasonic sensor script

		├── SensorROS/IMU/IMUPublisher.cs  // IMU script
		
		└── SensorROS/GPS/GPSPublisher.cs  // GPS script

    └── ...                                // check the rest files if you have more interests, 
                                            if you just want to use the simulation, the rest could be ignored.









