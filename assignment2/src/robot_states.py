#!/usr/bin/env python
"""
.. module:: robot_states
    :platform: Unix
    :synopsis: the robot_states python script in ros-moveit-opencv-ontology package

.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

Subscribes to:
    /odom
Uses Service:
    /state/set_battery_level
    /state/get_battery_level
    
    /state/get_pose
  
    /state/set_base_movement_state
  
    /state/get_base_movement_state
This node defines battery level, robot pose and robot base movement state in order to be used by other
nodes in the software architecture. 
"""
import rospy
from robot_control import logs_mapper as anm
from assignment2.msg import RoomConnection, Point
from assignment2.srv import GetPose, GetPoseResponse, GetBatteryLevel, SetBatteryLevel, GetBatteryLevelResponse, SetBatteryLevelResponse
from assignment2.srv import GetBaseMovementState, GetBaseMovementStateResponse, SetBaseMovementState, SetBaseMovementStateResponse
from nav_msgs.msg import Odometry

# A tag for identifying logs producer.
node_fsm = anm.NODE_ROBOT_STATE

class RobotState:
    """
    Initializes the "robot-states" node to provide crucial information about the robot's current 
    state, such as its pose, battery level, and base movement state.
    """
    def __init__(self):

        """
        Initializes the "robot-states"
        """
        # Initialize a robot 
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        self._pose = Point()
        self._battery_level = 1000
        self._base_movement_state = False
        # Define services.
        rospy.Service(anm.ROBOT_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.ROBOT_GET_BATTERY_LEVEL, GetBatteryLevel, self.get_battery_level)
        rospy.Service(anm.ROBOT_SET_BATTERY_LEVEL, SetBatteryLevel, self.set_battery_level)
        rospy.Service(anm.ROBOT_SET_BASE_MOVEMENT_STATE, SetBaseMovementState, self.set_base_movement_state)
        rospy.Service(anm.ROBOT_GET_BASE_MOVEMENT_STATE, GetBaseMovementState, self.get_base_movement_state)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        # Logs

        log_msg = (f'Initialise a robot `{anm.NODE_ROBOT_STATE}` with services `{anm.ROBOT_GET_POSE}` and '
                   f' `{anm.ROBOT_GET_BATTERY_LEVEL}` and `{anm.ROBOT_SET_BATTERY_LEVEL}` and '
                   f' `{anm.ROBOT_GET_BASE_MOVEMENT_STATE}` and `{anm.ROBOT_SET_BASE_MOVEMENT_STATE}`.')
        
        # print(log_msg)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))

        ### execution
        while not rospy.is_shutdown(): 
            if self._base_movement_state == True:
                self._battery_level -= 1
                print("Executed")
            rospy.sleep(1)

    def odom_callback(self, data):
        """
        Callback function for the /odom topic subscriber that updates the robot's current pose in the robot-states node.
        The argument for this function is the data of type nav_msgs.msg.Odometry.
        """
        self._pose.x = data.pose.pose.position.x
        self._pose.y = data.pose.pose.position.y

    def get_pose(self, request):
        """
        The implementation of the /state/get_pose service. The input parameter, request, 
        from the client is not used. The response returned to the client contains the current 
        robot pose. The argument request is of type GetPoseRequest. The returned value is of
        type GetPoseResponse.
        """
        if self._pose is None:
            # print(self._pose)

            ### rospy log
            rospy.logerr(anm.tag_log('Cannot get a robot position', node_fsm))
        else:

            #### get the position
            log_msg = f'Get a robot position via `{anm.ROBOT_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))

        ### execute pose    
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def set_battery_level(self, request):
        """
        The implementation of the state/set_battery_level service.
        The input parameter request is the current battery level of the robot to be set, 
        which is provided by the client. The server returns an empty response.
        """
        if request.battery_level is not None:

            print("Battery level is fine and set")
            self._battery_level = request.battery_level
            log_msg = (f'Set current robot battery level via `{anm.ROBOT_SET_BATTERY_LEVEL}` '
                             f'as ({self._battery_level}).')
            
            print(log_msg)
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        else:

            ### error appeard
            print('Error with battery')
            rospy.logerr(anm.tag_log('Couldnt set battery level', node_fsm))
        return SetBatteryLevelResponse()

    def get_battery_level(self, request):
        """
        The implementation of the state/get_battery_level service. The input parameter of 
        the request is empty, as provided by the client. The response returned to the client
        includes the current battery level of the robot. 
        
        Args: 
            request (GetBatteryLevelRequest). 
        Returns: 
            response (GetBatteryLevelResponse).
        """
        if self._battery_level is None:

            print('Error with battery')
            rospy.logerr(anm.tag_log('Could not get battery level', node_fsm))
        else:
            log_msg = f'Get battery level via `{anm.ROBOT_GET_BATTERY_LEVEL}` as ({self._battery_level})'

            print(log_msg)
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        
        ### response of getting battery
        response = GetBatteryLevelResponse()
        response.battery_level = self._battery_level
        return response

    def set_base_movement_state(self, request):
        """
            The `state/set_base_movement_state` service implementation.
            The `request` input parameter is the current robot base movement state to be set,
            as given by the client. This server returns an empty `response`.
            Arg:
                request(SetBaseMovementStateRequest)
        """
        if request.base_movement_state is not None:
            self._base_movement_state = request.base_movement_state
            log_msg = (f'Set current robot movement state through `{anm.ROBOT_SET_BASE_MOVEMENT_STATE}` '
                       f'as ({self._base_movement_state}).')
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot movement state', node_fsm))
        return SetBaseMovementStateResponse()

    def get_base_movement_state(self, request):
        """
        The implementation of the state/get_base_movement_state service. The request parameter provided by the client is empty 
        and is not used. The response returned to the client includes the current state of the robot's base movement.
        Args:
            request (GetBaseMovementStateRequest)

        Returns:
            response (GetBaseMovementStateResponse)
        """
        if self._base_movement_state is None:
            print("Error in base statement ")
            rospy.logerr(anm.tag_log('Could not get movement state', node_fsm))
        else:
            print("got it")
            log_msg = f'Get current robot movement state through `{anm.ROBOT_GET_BASE_MOVEMENT_STATE}` as ({self._base_movement_state})'
            print(log_msg)
            rospy.loginfo(anm.tag_log(log_msg, node_fsm))

        #### response of base movement
        response = GetBaseMovementStateResponse()
        response.base_movement_state = self._base_movement_state
        return response

# main
if __name__ == "__main__":
    print("Launching the Robot State")
    RobotState()
    rospy.spin()
