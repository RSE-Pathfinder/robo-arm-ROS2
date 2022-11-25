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

  // Declare File reference
  std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

  std::ifstream book_file(HOME + "/robo_arm_ws/src/robo-arm-ROS2/robo-arm-gazebo/models/environment/book.urdf");
  std::string book_object{};

  std::ifstream bookshelf_file(HOME + "/robo_arm_ws/src/robo-arm-ROS2/robo-arm-gazebo/models/environment/bookshelf.urdf");
  std::string bookshelf_object{};
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Here");

  if (book_file.is_open()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Book File is Open");
    std::string line;
    while (getline (book_file, line)) {
      book_object.append(line);
    }
    book_file.close();
  }

  if (bookshelf_file.is_open()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bookshelf File is Open");
    std::string line;
    while (getline (bookshelf_file, line)) {
      bookshelf_object.append(line);
    }
    bookshelf_file.close();
  }

  // Declaration of 3D space variables
  int pos_x = 0.0;
  int pos_y = 0.0;
  int pos_z = 0.0;

  // Initialise Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("book_spawner_client");

  // Initialise Client Node
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

  // Initialise Request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "Bookshelf";
  request->xml = bookshelf_object;
  request->robot_namespace = "environment";
  request->initial_pose.position.x = pos_x + 0.0;
  request->initial_pose.position.y = pos_y + 0.0;
  request->initial_pose.position.z = pos_z + 0.5;
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bookshelf Spawned");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
  }

  // Spawn Book 1
  request->name = "Book1";
  request->xml = book_object;
  request->robot_namespace = "environment";
  request->initial_pose.position.x = pos_x + -0.5;
  request->initial_pose.position.y = pos_y + -0.1;
  request->initial_pose.position.z = pos_z + 0.31;

  result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Book 1 Spawned");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
  }

  // Spawn Book 2
  request->name = "Book2";
  request->xml = book_object;
  request->robot_namespace = "environment";
  request->initial_pose.position.x = pos_x + -0.5;
  request->initial_pose.position.y = pos_y + 0.0;
  request->initial_pose.position.z = pos_z + 0.31;

  result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Book 2 Spawned");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
  }

  // Spawn Book 3
  request->name = "Book3";
  request->xml = book_object;
  request->robot_namespace = "environment";
  request->initial_pose.position.x = pos_x + -0.5;
  request->initial_pose.position.y = pos_y + 0.1;
  request->initial_pose.position.z = pos_z + 0.31;

  result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Book 3 Spawned");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service spawn_entity");
  }

  rclcpp::shutdown();

  return 0;
}
