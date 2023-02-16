# Robot Patrol

## Outline 

1. Introduction
2. Desrciption of software architecture 
   * Component diagram
   * state diagram 
3. Organization of project's files 
4. Instalation and running procedures
5. Working hypothesis and environment
   * System's features 
   * System's limitations
   * Possible technical improvements 
6. Doxygen documentation
7. Authors and Teachers contact  

# 1. Introduction 
Here is the assignment of Experimental Robotics course. The robot is patrolling around the corridors with help of ontology, smach, ros. Simulation of the robot is also represented. 
   1. Creat a robot model such that the robot can:
         * Navigate inside
         * Scan aruco codes by rgb camera
         * Detect obstacles by lidar sensor 
   2. Build a topological ontology strating from the data collected in aruco codes 
   3. Creat a state machine and update about its state
   
The finit-state machine is built in ROS environment based on [SMACH](http://wiki.ros.org/smach/Tutorials) ros-package.
Protogé is used for building the ontology [ARMOR](https://github.com/EmaroLab/armor_rds_tutorial) service is used for access and modify the ontology. 

#### Environment
The environment consists of four rooms (R1, R2, R3, R4), two corridors (C1, C2), a waiting room called "E" used before building the map, a charging station, and seven doors (D1 to D7).
 


#### Senario
 We have a survallience robot that moves in a 2D environment. First the robot waits in room **E** untill the whol map  is loaded into the ontology by scaning the arUco markers, then the robot keeps moving between the corridors **C1** and **C2** for infinit time. The robot must go to the charging station **E** when its battery is low, It must visit the **urgent** rooms if any, then goes back to its normal behavior moving between cooridors.
 
 #### Ontology
* Entity 
* Individuals 
  * `C1 C2 E`: Cooridors 
  * `R1 R2 R3 R4`: Rooms
  * `D1 D2 D3 D4 D5 D6 D7`: Doors
* Object properties
  * isIn
  * canReach
  * hasDoor 
  * connectedTo
* Data properties 
  * Now
  * visitedAt
  * uregencyThreshold
  

 # 2. Discreption of software architecture 
 ## Component diagram 
 ### 1- Marker publisher
A node subscribes to the topic of camera, then when a marker is detected its ID will be identified. It calls the marker_server_py server with the resquest of the ID of the scanned marker. It does some image processing when the marker is detected and show them.

 ### 2- Marker server: 
 It is a server that takes the request sent by the marker_publisher and for each specific ID it changes the topological map by calling armor server
 
 ### 3- state_machine:
It controlls how the robot changes its state based on the reasoning of the topological ontology, and the battery state of the robot. This node subscribes to two topics `/battery_state` and `/map_state`, and it calls the armor server for updating the loaded ontology. The states of the robot are listed bellow.
  * filling_map
  * moving_in_corridors
  * visiting_urgent
  * charging
 
 In addition to this, the node uses /move_base to send a goal for the robot to make it moving 
  
 ### 4- battery_controller 
 This is a publisher to the topic `/baterry_state`, it publishes different state of the battery `True`or`False` in a specific duration, the durations to be full or low are passed as **parameters**.
 
 ### 5- armor
 The ARMOR package is an external package used to communicate with the Cluedo OWL ontology, it is a ROS wrapper that helps to make this communication possible. For more information about ARMOR [click here](https://github.com/EmaroLab/armor_rds_tutorial)
 
 ### 5- SLAM
 Building the map, localizing the robot and navigation based on move_base action server
 
 ### List of parameters 
 It is a **yaml** file that list all the parameters used in this project which are 
 * time_to_stay_in_location defeault value = 1 (s)
 * time_of_one_charge  defeault value = 30 (s)
 * time_of_charging  defeault value = 15 (s)
 * visitedAt_R1  defeault value = 1665579740
 * visitedAt_R2  defeault value = 1665579740
 * visitedAt_R3  defeault value = 1665579740
 * visitedAt_R4  defeault value = 1665579740
 * visitedAt_C1  defeault value = 1665579740
 * visitedAt_C2  defeault value = 1665579740
 * visitedAt_E  defeault value = 1665579740
### List of messages 
* Map_state.msg: `int32 map_state`, carry the state of the map
  * It is 1 if the map is fully loaded 
  * It is 0 if the map is not fully loaded
* Battry_state.msg: `int32 battery_state`, carry the state of the battery
  * It is 1 if the battery is full
  * It is 0 if the battery is low
* RoomConnection
  * connected_to
  * through_door

### List of messages 
* RoomInformation 

## State diagram 

There are four states in this state diagram, the task of each state is explained below:

1. **Filling_Map**: The robots keeps waiting in room **E** untill the whole map is loaded, the robot can know if the map is fully loaded by subscribing to the topic `/map_state`. When the map is fully loaded the robot go to `MOVING_IN_CORRIDORS` state trough the transition `move`.
2. **Moving in corridors**: The robots keeps moving between `C1` and `C2` for infinit time, the robots keep cheking the state of the battery by subscribing to the topic `/battery_state`, if the battery is `low` the robot goes to `CHARGING` state trough the transition `tired`. if the battery is not low, the robot cheks if there is an `urgent` room by quering the `individuals of the class urg`. If there is an urgent room the robot goes to `VISITING_URGENT` state through the transition `urgent`.
3. **Visiting_urgent**:  First, the robots checks if the battery is low goes to `CHARGING` state, else it checkes it the urgent room is reacheable by quering the object properties of the robot `canReach`, if the room can be reached it visit it and return back to the `MOVING_IN_CORRIDORS` trough the transition `visited`. If the urgent room is not reacheable it goes to `MOVING_IN_CORRIDORS` state through the transition `not_reached`, in this case the robot will change the corridors and visit it again.
4. **CHARGING**: The robot keeps checking the state of the battery, if it is full it goes to `MOVING_IN_COORIDORS` state the the transition `charged`, otherwise, it stays in the `CHARGING` state. 

# 4. Instalation and Running

### Installation
* The usage of this package requires the prior installation of [aRMOR](https://github.com/EmaroLab/armor) as described in the linked resource.

* Install aruco and upload all model to .gazebo/models:

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

# 5. Working hypothesis and environement
In this project there are many assumptions made on the environement in order to make the project simpler, the assumptions are explained bellow
1. The movement of the robot from one location to another location is considered to be only one time step in the onotology, it means that if the robot takes 2 minutes to go from R1 to C1, this will be considered as one time step on the ontology
3. Initializing the `visitedAt` data proporty of a location to different values to avoid making all of them **urgent** at the same time 
4. When a robot is in a specific location, it can reach only the locations that need **one door** transmition

## Possible improvements  
1. System Capabilities: This package utilizes a URDF robot equipped with a camera mounted on a 5 degree of freedom manipulator to detect markers. The robot can also generate a map in real-time and find the optimal path to the target point.

2. System Limitations: The main limitation of this experiment is the battery level, which was selected to be high enough for the robot to effectively patrol the environment.

3. Potential Technical Enhancements: The design of the robot could be improved to provide better balance between the base and manipulator to avoid swaying in real-world scenarios. Additionally, the ontology aspect of the system could be expanded to take full advantage of the power of the PELLET reasoner.

# 6. Doxygen documentation

# 7. Author and Teachers contacts 
* Author 
  * name: Bauyrzhan Zhakanov
  * email: bauyrzhan.zhakanov@gmail.com
* Teachers
  * name: Luca Buoncompagni luca.buoncompagni@edu.unige.it
  * name: Carmine Recchiuto carmine.recchiuto@dibris.unige.it

 

