#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include <fstream>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

/* MAIN */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Declare XML File reference
  std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
  std::ifstream xml_file(HOME + "/robo_arm_ws/src/robo-arm-ROS2/robo-arm-gazebo/models/environment/book.urdf");
  std::string xml_object{};
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Here");

  if (xml_file.is_open()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "XML File is Open");
    std::string line;
    while (getline (xml_file, line)) {
      xml_object.append(line);
    }
    xml_file.close();
  }

  // Initialise Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("book_spawner_client");

  // Initialise Client Node
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

  // Initialise Request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "Book1";
  request->xml = xml_object;
  request->robot_namespace = "book";
  request->initial_pose.position.x = 0.0;
  request->initial_pose.position.y = 0.0;
  request->initial_pose.position.z = 1.0;
  // request->initial_pose.orientation.x = 0.0;
  // request->initial_pose.orientation.y = 0.0;
  // request->initial_pose.orientation.z = 0.0;
  // request->initial_pose.orientation.w = 1.0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Books Spawned");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
  }

  rclcpp::shutdown();

  return 0;
}
