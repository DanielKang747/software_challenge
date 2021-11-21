#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("reset_turtle_client");
	rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client = node->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");
	
	auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

	request->x = 25;
	request->y = 10;
	request->theta = M_PI/2;

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
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset turtle");
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call reset_turtle");
	}
	
	rclcpp::shutdown();
	return 0;
}
