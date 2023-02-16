# Robot Patrol

## Video representation and [Documentation]()

## 1. Introduction 
The following is the Experimental Robotics course project description, where a robot is utilized to patrol corridors through the implementation of ontology, smach, and ros. Additionally, the project includes the simulation of the robot's behavior. The project involves the following tasks:

   - Develop a robot model capable of:
        - Navigating indoors
        - Scanning aruco codes using an RGB camera
        - Detecting obstacles with a lidar sensor

   - Construct a topological ontology based on the data acquired from the aruco codes.

   - Create a state machine and keep track of its status updates.
   
The finite-state machine is built in ROS environment based on [SMACH](http://wiki.ros.org/smach/Tutorials) ros-package.
Protogé is used for building the ontology [ARMOR](https://github.com/EmaroLab/armor_rds_tutorial) service is used for access and modify the ontology. 

### Environment
Within the given setting, there are four distinct rooms labeled as R1, R2, R3, and R4, along with two corridors labeled as C1 and C2. Additionally, there is a waiting room known as "E," which was utilized prior to creating the map. The environment also contains a charging station and seven doors identified as D1 to D7.

#### Topological Map and GAZEBO simulation

<p float="center">
  <img src="https://github.com/BZWayne/robotPatrol/blob/main/images/enviroment_ontology.png" width="340" />
  <img src="https://github.com/BZWayne/robotPatrol/blob/main/images/environment.png" width="600" /> 
</p>


### Senario
A 2D environment is being monitored by a surveillance robot. Initially, the robot is stationed in room E and awaits until the entire map has been loaded into the ontology by scanning the arUco markers. Afterward, the robot continually traverses between corridors C1 and C2. Once the robot's battery is low, it must travel to the charging station located at E. If any urgent rooms require attention, the robot must visit them before resuming its routine behavior of moving between corridors.
  
## 2. Discription of software architecture 
 
### Component and Temporal Diagram of Software Architecture

#### Component Diagram

#### Temporal Diagram

#### 1- Marker Publisher 
[OpenCV](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge) is utilized for the image processing aspect of the task, employing the method presented in [aruco](https://github.com/CarmineD8/aruco_ros). To enable conversion between ROS Image messages and OpenCV images, the cv_bridge package is utilized.

#### 2- Marker Server:
This implementation provides a service that responds with information for each room in the semantic map. It does so by receiving a call request with an argument specifying the room ID.

#### 3- State Machine:
The [SMACH](http://wiki.ros.org/smach) state machine controls the robot's state based on the topological ontology map reasoning and the robot's battery status. The following are the robot's states:

    - Load Map
    - Moving in Corridors
    - Discover Room
    - Moving for Battery
    - Charging
 
There are 5 states in this state diagram, the task of each state is explained below:
1. **LoadMap**: The initial state is defined as building a semantic map for the robot. The arm of the robot is set to move along desired trajectories as defined in the robotik.cpp file using the setArmMotion(data) function. If sufficient room IDs are detected, the state will exit and return the status of map_loaded.
    
2. **MovinginCorridors**: The state where the robot moves to a desired location is defined in this code. First, the function set_base_movement_state(base_movement_state) is used to enable battery consumption. Then, the robot moves to the target room with the help of the function 
movingPose(pose). As the robot moves to its destination, the ontology is updated with the function update_ontology(now). When the robot reaches its target room, the state is exited and the function returns reached. If the battery level drops below the threshold, the function returns battery_low and the target room is cancelled.

3. **Discover Room**: The sate defines the state where the robot has arrived at the target room and begins to explore it. The robot arm's movement is enabled just like in the initial state using the setArmMotion(data) function, and then it returns discovered.

4. **MovingforBattery**: The state when the battery is low and the robot needs to recharge is defined. The function set_base_movement_state(movement_state) is used to enable battery consumption, and the robot moves towards the charger using the movingPose(pose) function. The ontology is updated during the movement to the charger using the update_ontology(time) function until the robot reaches the charger."

5. **Charging**: The state is called Charging() and it activates the robot has reached the charging station and recharges the battery. The battery level is updated using the set_battery_level(battery_level) function after a certain period of time has elapsed. The state then transitions to charged.
 
#### 4- moveit
The robot manipulator arm is controlled by a controller node. The node is implemented by the robotik package, which is auto-generated by [MoveIt](https://moveit.ros.org/) for the URDF of the robot. This node provides joint trajectory effort controllers for the robot arm, allowing it to move to the desired joint configurations.

##### 4.1 - robot model
 
#### 5- SLAM gmapping
[slam_gmapping](https://github.com/ros-perception/slam_gmapping), a popular ROS package, is utilized for the purpose of creating maps. By utilizing data from a robot laser scanner, in conjunction with the robot's base frame position, slam_gmapping is able to locate the robot within the map. Unlike other mapping packages, slam_gmapping generates the map in real-time, as it does not require a pre-existing map file to function.


## 4. Instalation and Running

### Installation

My version of ROS is Noetic, but it should work for other versions too.

* The usage of this package requires the prior installation of [aRMOR](https://github.com/EmaroLab/armor) as described in the linked resource.

* Install [aruco](https://github.com/CarmineD8/aruco_ros) and upload all model to **.gazebo/models**:

* Install slam_gmapping:

```bashscript
$ sudo apt install ros-<distro>-gmapping
```

* Install [OpenCV](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge):

* Install konsule:

```bashscript
$ sudo apt-get install konsole
```

* Install smach state for ROS if not installed:
```bashscript
$ sudo apt-get install ros-<distro>-executive-smach*
```
```bashscript
$ sudo apt-get install ros-<distro>-smach-viewer
```

### Running

```bashscript
$ roscd assignment2/launch
```
```bashscript
$ bash run.sh
``` 

## Future Work
1. **System Capabilities**: This package utilizes a URDF robot equipped with a camera mounted on a 5 degree of freedom manipulator to detect markers. The robot can also generate a map in real-time and find the optimal path to the target point.
2. **System Limitations**: The main limitation of this experiment is the battery level, which was selected to be high enough for the robot to effectively patrol the environment.
3. **Potential Technical Enhancements**: The design of the robot could be improved to provide better balance between the base and manipulator to avoid swaying in real-world scenarios. Additionally, the ontology aspect of the system could be expanded to take full advantage of the power of the PELLET reasoner.


## Author and Teachers contacts 
* Author 
  * Bauyrzhan Zhakanov, *s5218116@studenti.unige.it*
* Teachers
  * Luca Buoncompagni luca.buoncompagni@edu.unige.it
  * Carmine Recchiuto carmine.recchiuto@dibris.unige.it
  
