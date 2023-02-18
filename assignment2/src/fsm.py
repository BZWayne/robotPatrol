#!/usr/bin/env python
"""
	.. module:: fsm
		:platform: Unix
		:synopsis: the fsm python script

	.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

	Subscribes to:
		/image_id
	Uses Service:
		/state/set_battery_level
		/state/get_pose
		/state/set_base_movement_state
		/room_info
		/move_arm
	Uses Action:
		/move_base
	Uses helper script:
		/utilities/robot_control/map.py

	The finite state machine's initial state involves building a semantic map of
	the environment using image IDs of markers detected by the robot camera. 
	This node updates the ontology using the map.py helper script 
	and retrieves the target room based on the last visit times and the robot's 
	battery state. In the next state, the robot moves to the target room, and if 
	the battery level drops below a threshold, the robot goes to the charger to recharge.
"""
import rospy
import smach
import smach_ros
import math
import numpy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment2.msg import RoomConnection, Point
from assignment2.srv import SetBatteryLevel, GetPose, SetBaseMovementState, RoomInformation
from robot_control import logs_mapper as anm
from robot_control.map import MapOntology
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from threading import Lock 

# A tag for identifying logs producer.
node_fsm = anm.NODE_FINITE_STATE_MACHINE
node_time_for_log = 5

publisher_ = None
ontology = None
threading_lock = None
room_id = []
room_name = []
room_center = []
room_connection = []
door = []

def getRoomId(data):
    """
    The client for the marker_server retrieves information for a specific room using the room_info service.
    The argument room_id is an integer that is used to identify the room. The function returns a response 
    in the form of resp (RoomInformationResponse)."
    """
    rospy.wait_for_service('room_info')
    try:
        service = rospy.ServiceProxy('room_info', RoomInformation)
        resp = service(data)
        return resp 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def getImageId(data):
    """
    The callback function for the subscriber of the /image_id topic checks if the image id detected 
    is valuable and has not already been recorded. If so, it saves the information for each room by 
    calling the function getRoomId(room_id) and modifies the ontology using functions from the helper 
    script map.py such as add_room(room), add_door(door), assign_doors_to_room(room, doors), 
    disjoint_individuals(), and add_last_visit_time(room, visit_time). The argument data is an 
    integer that represents the image id.
    """
    global room_id
    global room_name
    global room_center
    global room_connection
    global door

    if data.data not in room_id and data.data > 10 and data.data < 18:
        room_id.append(data.data)    
        room_info = getRoomId(data.data)
        room_name.append(room_info.room)
        log_msg = 'Semantic map updated, room '+ room_info.room + ' detected'
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        ontology.add_room(room_info.room)

        room_center.append([room_info.x, room_info.y])
        log_msg = 'Center position is: [%f, %f]' % (room_info.x, room_info.y)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))

        for i in range(len(room_info.connections)):
            room_connection.append(room_info.connections[i].connected_to)
            door.append(room_info.connections[i].through_door)
            log_msg = 'Room ' + room_info.room + ' is connected to ' + room_info.connections[i].connected_to + ' via door ' + room_info.connections[i].through_door
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))
            ontology.add_door(room_info.connections[i].through_door)
            ontology.assign_doors_to_room(room_info.room, room_info.connections[i].through_door)

        ontology.disjoint_individuals()
        ontology.add_last_visit_time(room_info.room, str(room_info.visit_time))
        

def movingPose(data):
    """
    The function serves as an action client for the "move_base" node. It takes a pose as an argument 
    and sends it in the form of "MoveBaseGoal.msg" to the action server. The function returns the 
    result in the form of "MoveBaseResult.msg".
    """
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = data.x
    goal.target_pose.pose.position.y = data.y
    goal.target_pose.pose.orientation.w = 1.0
    client.wait_for_server()
    client.send_goal(goal)

def getConfirmation(data):
    """
    Determines if the robot has arrived at a specific point by calculating the Euclidean distance 
    between the robot's current pose and the target pose.

    Args:
        target_pose (Point): The desired location for the robot to reach.

    Returns:
        reached (Bool): Indicates whether or not the robot has arrived at the target pose.
    """
    rospy.wait_for_service(anm.ROBOT_GET_POSE)
    try:
        service = rospy.ServiceProxy(anm.ROBOT_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        if math.sqrt((data.x - pose.x)**2 + (data.y - pose.y)**2) < 1:
            reached = True
        else:
            reached = False
        log_msg = 'target reached state: ' + str(reached)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        return reached
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

def getRoomLocation(data):
    """
    This function detects the center position of a specific room by using the room information.

    Args:

        room (string): the name of the room to detect the center position of

    Returns:

        room_pose (Point): the center position of the specified room
    """
    global room_name
    global room_center
    room_pose = Point()
    room_index = room_name.index(data)
    room_pose.x = room_center[room_index][0]
    room_pose.y = room_center[room_index][1]
    return room_pose

def battery_data(data):
    """
    Updates the battery level of the robot by using the /state/battery_data service.
    The current level of the robot's battery is stored in the robot-state node.
    """
    rospy.wait_for_service(anm.ROBOT_SET_BATTERY_LEVEL)
    try:
        log_msg = f'Set a battery level to the `{anm.ROBOT_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        service = rospy.ServiceProxy(anm.ROBOT_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(data)
    except rospy.ServiceException as e:
        log_msg = f'Cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

def setArmMotion(data):
    """
    Updates the arm movement state of the robot through the service client for /move_arm 
    stored in the robotik node.
    """
    rospy.wait_for_service('/move_arm')
    try:
        log_msg = 'Moving an arm to ' + str(data)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        service = rospy.ServiceProxy('move_arm', SetBool)
        service(data)
    except rospy.ServiceException as e:
        log_msg = f'Cannot set a arm movement: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

def base_data(data):
    """
    Service client function for /state/set_base_movement_state. Modifies the robot's base movement 
    state stored in the robot-states node.
    """
    rospy.wait_for_service(anm.ROBOT_SET_BASE_MOVEMENT_STATE)
    try:
        log_msg = 'Setting base movement state to ' + str(data)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        service = rospy.ServiceProxy(anm.ROBOT_SET_BASE_MOVEMENT_STATE, SetBaseMovementState)
        service(data)  
    except rospy.ServiceException as e:
        log_msg = f'Cannot set base movement: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

class LoadMap(smach.State):
    """
    The initial state is defined as building a semantic map for the robot. The arm of the robot is set to move along
    desired trajectories as defined in the robotik.cpp file using the setArmMotion(data) function. If sufficient room 
    IDs are detected, the state will exit and return the status of map_loaded.
    """
    def __init__(self):
        """
        Initialize the execution with map_loaded state
        """
        smach.State.__init__(self, outcomes=['map_loaded'])

    def execute(self, userdata):
        """
        Execute the LoadMap() state
        """
        global threading_lock
        global room_id

        ### execution
        setArmMotion(True)
        while not rospy.is_shutdown(): 
            threading_lock.acquire()
            try:
                if len(room_id) > 6:
                    return 'map_loaded'
            # except:
            #     return 'map_not_loaded'
            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class MovingCorridor(smach.State):
    """
    The state where the robot moves to a desired location is defined in this code. 
    First, the function set_base_movement_state(base_movement_state) is used to enable battery 
    consumption. Then, the robot moves to the target room with the help of the function 
    movingPose(pose). As the robot moves to its destination, the ontology is updated with 
    the function update_ontology(now). When the robot reaches its target room, the state is 
    exited and the function returns reached. If the battery level drops below the threshold, 
    the function returns battery_low and the target room is cancelled.
    """
    def __init__(self):
        """
        Initialize the execution with reached or battery_low states
        """
        smach.State.__init__(self, outcomes=['reached', 'battery_low'])

    def execute(self, userdata):
        """
        Excecute MovintCorridor() state
        """
        global threading_lock
        global ontology

        ## parameters for ontology
        now = rospy.get_rostime()
        [target_room, battery_low] = ontology.update_ontology(now)
        target_room_pose = getRoomLocation(target_room)
        base_data(True)
        movingPose(target_room_pose)

        ## execution
        while not rospy.is_shutdown(): 
            threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                [next_target_room, battery_low] = ontology.update_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, node_fsm))
                if battery_low:
                    return 'battery_low'
                else:
                    reached = getConfirmation(target_room_pose)
                    if reached:
                        base_data(False)
                        return 'reached'
            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class DiscoverRoom(smach.State):
    """
    Defines the state where the robot has arrived at the target room and begins to explore it. 
    The robot arm's movement is enabled just like in the initial state using the setArmMotion(data) 
    function, and then it returns discovered.
    """
    def __init__(self):
        """
        Initialize the execution with discovered state
        """
        smach.State.__init__(self, outcomes=['discovered'])

    def execute(self, userdata):
        """
        Executes DiscoverRoom() state
        """
        global threading_lock
        global ontology

        ## execution
        while not rospy.is_shutdown(): 
            threading_lock.acquire()
            try:
                setArmMotion(True)
                return 'discovered'

            finally:
                threading_lock.release()

class MovingForBattery(smach.State):
    """
    The state when the battery is low and the robot needs to recharge is defined. 
    The function set_base_movement_state(movement_state) is used to enable battery 
    consumption, and the robot moves towards the charger using the movingPose(pose) 
    function. The ontology is updated during the movement to the charger using the 
    update_ontology(now) function until the robot reaches the charger."

    """
    def __init__(self):
        """
        Initialize the execution with reached state
        """
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        """
        Executes MovingForBattery() state
        """
        global threading_lock
        global ontology

        ## parameters for ontology
        now = rospy.get_rostime()
        ontology.update_ontology(now)
        target_room_pose = getRoomLocation('E')
        base_data(True)
        movingPose(target_room_pose)

        ## execution
        while not rospy.is_shutdown():  
            threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                ontology.update_ontology(now)
                reached = getConfirmation(target_room_pose)
                if reached:
                    base_data(False)
                    return 'reached'              

            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class Charging(smach.State):
    """
    The state is called Charging() and it activates the robot has reached the charging station and 
    recharges the battery. The battery level is updated using the set_battery_level(battery_level) 
    function after a certain period of time has elapsed. The state then transitions to charged.
    """
    def __init__(self):
        """
        Initialize the execution with charged state
        """
        smach.State.__init__(self, outcomes=['charged'])

    def execute(self, userdata):
        """
        Executes Charging() state
        """
        global threading_lock
        global ontology
        
        ### execution
        while not rospy.is_shutdown():  
            threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                ontology.update_ontology(now)
                rospy.sleep(10)
                battery_data(1000)
                return 'charged'

            finally:
                threading_lock.release()

def main():
    """
    The main function for the finite_state_machine node initializes the node and takes an instance of the
    MapOntology class at the current time. It defines a subscriber for the /image_id topic and outlines the
    states and transitions for the finite state machine's topological map. Finally, it starts the finite state machine process.
    """
    # Initialise this node.
    global ontology
    global publisher_
    global threading_lock

    rospy.init_node(node_fsm, log_level=rospy.INFO)
    now = rospy.get_rostime()
    ontology = MapOntology(node_fsm, now)

    # Get or create a new threading_lock.
    if threading_lock is None:
        threading_lock = Lock()
    else:
        threading_lock = threading_lock

    # Subscribe image id to get rooms information
    rospy.Subscriber('/image_id', Int32, getImageId)

    # Create a SMACH state machine
    robot_sm = smach.StateMachine(outcomes=[])
    robot_sm.userdata.sm_counter = 0

    # Open the container
    with robot_sm:
        # Add states to the container
        smach.StateMachine.add('Load_Map', LoadMap(), 
                                transitions={'map_loaded':'Moving_Corridor'})
        smach.StateMachine.add('Moving_Corridor', MovingCorridor(), 
                                transitions={'battery_low':'Moving_For_Battery', 'reached':'Discover_Room'})
        smach.StateMachine.add('Moving_For_Battery', MovingForBattery(), 
                                transitions={'reached':'Charging'})
        smach.StateMachine.add('Discover_Room', DiscoverRoom(), 
                                transitions={'discovered':'Moving_Corridor'})
        smach.StateMachine.add('Charging', Charging(), 
                                transitions={'charged':'Moving_Corridor'})

    # Create and start the introspection for visualization
    sis = smach_ros.IntrospectionServer('server_name', robot_sm, '/SM_ROOT')
    sis.start()

    # Execute the smach
    outcome = robot_sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
