#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("clear_client");
	rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client = node->create_client<turtlesim::srv::Kill>("/kill");

	auto request = std::make_shared<turtlesim::srv::Kill::Request>();
	
	request->name = "turtle1";

	while(!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) 
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Killed all turtles.");
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call spawn_turtles");
	}
	
	rclcpp::shutdown();
	return 0;
}

