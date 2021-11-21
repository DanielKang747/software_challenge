#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("spawn_turtles_client");
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client = node->create_client<turtlesim::srv::Spawn>("/spawn");
	
	auto request1 = std::make_shared<turtlesim::srv::Spawn::Request>();
	auto request2 = std::make_shared<turtlesim::srv::Spawn::Request>();

	request1->x = 25;
	request1->y = 10;
	request1->theta = M_PI/2;
	request1->name = "moving_turtle";
	
	request2->x = 5;
	request2->y = 5;
	request2->theta = M_PI/2;
	request2->name = "stationary_turtle";

	while(!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto result1 = client->async_send_request(request1);
	auto result2 = client->async_send_request(request2);

	if (rclcpp::spin_until_future_complete(node, result1) == rclcpp::FutureReturnCode::SUCCESS && rclcpp::spin_until_future_complete(node, result2) == rclcpp::FutureReturnCode::SUCCESS) 
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spawned a turtle: %s", result1.get()->name);
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call spawn_turtles");
	}
	
	rclcpp::shutdown();
	return 0;
}
