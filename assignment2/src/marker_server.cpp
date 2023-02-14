/**
 * @file marker_server.cpp
 * @brief This node implements a service to provide information of each room id
 * 
 * @details
 * 
 * Services: <BR>
 *	/room_info
 * 
 * Description:
 * 
 * This node implements a service: it requires the id (marker) detected by the 
 * robot and it replies with the information about the corresponding room (name of
 *  the room, coordinates of the center, connections with other rooms)
 */

#include <ros/ros.h>
#include <assignment2/RoomConnection.h>
#include <assignment2/RoomInformation.h>

/**
 * @brief Callback function for ``room_info`` service.
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool markerCallback(assignment2::RoomInformation::Request &req, assignment2::RoomInformation::Response &res){
	assignment2::RoomConnection conn;
	double now = ros::Time::now().toSec();
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		res.visit_time = now - 5;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		res.visit_time = now - 10;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		res.visit_time = now - 15;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		res.visit_time = now - 40;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		res.visit_time = now - 35;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		res.visit_time = now - 25;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		res.visit_time = now - 20;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		break;
	default:
		res.room = "No room associated with this marker id";
	}
	return true;
}	

/**
 * @brief main function for ``marker_server`` node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "assignment2");
	ros::NodeHandle nh;
	ros::ServiceServer oracle = nh.advertiseService( "/room_info",markerCallback);
	ros::spin();
	ros::shutdown();
	return 0;
}
