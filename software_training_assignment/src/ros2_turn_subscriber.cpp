#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TurnPublisher : public rclcpp::Node
{
	public:
		TurnPublisher()
		: Node("turn_publisher")
		{
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/moving_turtle/cmd_vel", 1000);
			timer_ = this->create_wall_timer(500ms, std::bind(&TurnPublisher::timer_callback, this));
		}
	
	private:
		void timer_callback()
		{
			auto message = geometry_msgs::msg::Twist();
			message.linear.x = 2;
			message.angular.z = 1.8;
			RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.angular.z);
			publisher_->publish(message);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurnPublisher>());
	rclcpp::shutdown();
	return 0;
}
