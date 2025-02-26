# Hexapod ROS2

## Authors  

* **Andres Camilo Torres Cajamarca** – *Mechatronic Engineering Student*  
* **Felipe Chaves Delgadillo** – *Mechatronic Engineering Student*  

## Description  

ROS 2 Humble packages for controlling an 18-DOF (Degrees of Freedom) hexapod robot.  

## Usage  

To use these packages, you must install the following prerequisites:  

* [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)  
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)  
* [DynamixelSDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/)  

Then, run the following commands in an Ubuntu terminal:  

```bash
git clone https://github.com/antorresca/Hexapod
cd Hexapod
colcon build
source install/setup.bash
```

With this, you can use the packages by following the User's Manual.