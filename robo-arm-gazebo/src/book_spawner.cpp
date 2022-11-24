#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

/* MAIN */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Initialise Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("book_spawner_client");

  // Initialise Client Node
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client = node->create_client<gazebo_msgs::srv::SpawnEntity>("book_spawner");

  // Initialise Request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  //request = book;

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
