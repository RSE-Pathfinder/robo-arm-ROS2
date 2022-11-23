#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

//Logger for robo-arm-commander
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robo-arm-commander");

int main(int argc, char** argv){

rclcpp::init(argc, argv);
rclcpp::NodeOptions node_options;
node_options.automatically_declare_parameters_from_overrides(true);
auto move_group_node = rclcpp::Node::make_shared("robo-arm-commander", node_options);

//Spin up a SingleThreadedExecutor for the current state monitor to get information
//about the robot's state.
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(move_group_node);
std::thread([&executor]() { executor.spin(); }).detach();

//MoveIt operates on "planning groups" used interchangably with "joint model group"
static const std::string PLANNING_GROUP = "robo-arm";
//Create the move group interface object
moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
//Planning Scene interface object for adding and removing collision objects in the "virtual world"
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
// Raw pointers are frequently used to refer to the planning group for improved performance.
const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//Visualisation
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node,
    "robo_arm_link0", "robo-arm-commander", move_group.getRobotModel());

visual_tools.deleteAllMarkers();

//Remote control allows program stepping through buttons and key presses
visual_tools.loadRemoteControl();


}//main