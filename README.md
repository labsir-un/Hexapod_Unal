# Hexapod ROS 2  

## Authors  

* **Andres Camilo Torres Cajamarca** – *Mechatronics Engineering Student*  
* **Felipe Chaves Delgadillo** – *Mechatronics Engineering Student*  

## Tutors  

* **Ph.D. Eng. Pedro Fabián Cárdenas Herrera**  
* **Ph.D. Eng. Ricardo Emiro Ramírez Heredia**  

## Description  

ROS 2 Humble packages for controlling an 18-DOF (Degrees of Freedom) hexapod robot.  

## Installation and Usage  

Before using these packages, install the following prerequisites:  

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

After completing these steps, you can use the packages by following the User Manual.

## User Manual

### Connection with the Robot  

### Understanding ROS 2 Nodes, Topics, Services, and Actions  

### Launching ROS 2 Nodes  

#### Using Laptop

The following ROS 2 nodes must be launched to use the robot:

* *gui_node*
* *cinematica_node*
* *transformation_node*
* *dynamixel_node*

```bash
ros2 run gui_node gui_client
```

```bash
ros2 run transformation_node transformation_node 
```


```bash
ros2 run cinematica_node cinematica_node 
```


```bash
ros2 run dynamixel_node dynamixel_node 
```

Notes:

* Ensure that all nodes are developed. The *gui_node* will remain in a loop until *transformation_node* is launched, and *transformation_node*  will stay in a loop until *cinematica_node* is launched. 
* If the U2D2 is not connected, *dynamixel_node* will not work and will attempt to reconnect every 10 seconds.
* Always check the *logger* for information about the status and functionality of the nodes.

In *gui_node*, when you type **'1'**, the program will start, and the trajectory will be calculated in *cinematica_node*.  

Then, *transformation_node* will retrieve the first array of positions and attempt to send it to *dynamixel_node* via an action.  

Once *dynamixel_node* reaches all the positions, *transformation_node* will retrieve the next position, repeating the process until the user stops it.

**PRUEBA**

#### Automatic
