/**
 * @file robotik.cpp
 * @brief Implements a node to control the arm of robot using moveit
 * 
 * 
 * @details
 *
 * Services: <BR>
 *  /move_arm
 *
 * Description:
 * 
 * There are 7 markers in the environment, the joints configuration for each marker is found and once
 * the ``move_arm`` service is called with arg true, ``reach(req, resp)`` function tries to find the 
 * plan to the corresponding joints configuration and executes it.
**/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/SetBool.h>

/**
 * @brief Function for planning and executing it to each joints configuration. It uses the provided methods
 *  from ``moveit`` package for loading robot model, its kinematic and joints group. 
 * 
 * @param req 
 * @param resp 
 * @return true 
 * @return false 
 */
bool reach(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  // 1st pose  
  group.setStartStateToCurrentState();
  std::vector<double> joint_values;
  joint_values = {1.87, 1.0, -0.75, -1.77, 0.4};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 1 " << std::endl;
  sleep(3.0);

  // 2nd pose  
  group.setStartStateToCurrentState();
  joint_values = {1, -0.45, 0.45, 0.75, 0.0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 2 " << std::endl;
  sleep(3.0);

  // 3rd pose  
  group.setStartStateToCurrentState();
  joint_values = {-1.27, -0.5, -1.75, -1.27, 0.78};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 3 " << std::endl;
  sleep(3.0);

  // 4th pose  
  group.setStartStateToCurrentState();
  joint_values = {0.0, 0.0, 1.27, -1.57, 0.0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 4 " << std::endl;
  sleep(3.0);

  // 5th pose  
  group.setStartStateToCurrentState();
  joint_values = {-1.57, 0.75, 1.25, 0.0, 0.0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 5 " << std::endl;
  sleep(3.0);

  // 6th pose  
  group.setStartStateToCurrentState();
  joint_values = {1.57, -0.5, -1.75, -1.57, 0.78};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 6 " << std::endl;
  sleep(3.0);

  // 7th pose  
  group.setStartStateToCurrentState();
  joint_values = {0.0, 0.0, 0.0, 3.14, 0.0};
  group.setJointValueTarget(joint_values);
  group.setStartStateToCurrentState();
  // Plan and execute
  group.plan(my_plan); 
  group.execute(my_plan);
  std::cout << "Position 7 " << std::endl;
  sleep(3.0);

  resp.message = "ok";
  return true;

}

/**
 * @brief main function for ``robotik`` node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "robotik");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return(0);
}
  
  
  

