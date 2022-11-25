#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

//Logger for robo-arm-commander
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robo_arm_commander");

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::NodeOptions node_options;
RCLCPP_INFO(LOGGER, "Initialize node");

// This enables loading undeclared parameters
// best practice would be to declare parameters in the corresponding classes
// and provide descriptions about expected use
node_options.automatically_declare_parameters_from_overrides(true);
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("robo_arm_commander", "", node_options);

// We spin up a SingleThreadedExecutor for the current state monitor to get information
// about the robot's state.
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread([&executor]() { executor.spin(); }).detach();

// Setup
// ^^^^^
//
static const std::string PLANNING_GROUP = "robo_arm";
static const std::string LOGNAME = "robo_arm_commander";

/* Otherwise robot with zeros joint_states */
rclcpp::sleep_for(std::chrono::seconds(1));

RCLCPP_INFO(LOGGER, "Starting Robo Arm Commander...");

auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
// moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

// auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
// auto robot_start_state = planning_components->getStartState();
auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

// Plan #1
// ^^^^^^^
//
// We can set the start state of the plan to the current state of the robot
// planning_components->setStartStateToCurrentState();

// The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
geometry_msgs::msg::PoseStamped target_pose1;
target_pose1.header.frame_id = "panda_link0";
target_pose1.pose.orientation.w = 1.0;
target_pose1.pose.position.x = 0.35;
target_pose1.pose.position.y = 0.007;
target_pose1.pose.position.z = 0.006;
// planning_components->setGoal(target_pose1, "panda_link8");

// Now, we call the PlanningComponents to compute the plan and visualize it.
// Note that we are just planning
// auto plan_solution1 = planning_components->plan();

// Check if PlanningComponents succeeded in finding the plan
// if (plan_solution1)
// {
// Execute the plan
// planning_components->execute(); 
// }

}//main