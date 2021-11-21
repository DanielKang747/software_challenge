#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LocationSubscriber : public rclcpp::Node
{
	public: 
		LocationSubscriber()
		: Node("location_subscriber")
		{
			stationary_turtle_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/stationary_turtle/pose", 10, std::bind(&LocationSubscriber::stationary_callback, this, _1));
			moving_turtle_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", 10, std::bind(&LocationSubscriber::moving_callback, this, _1));
			publisher_ = this->create_publisher<turtlesim::msg::Pose>("/difference", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&LocationSubscriber::calculateDistance, this));
		}
	private:
		
		float xPosition_stationary = 0;
		float yPosition_stationary = 0;

		float xPosition_moving = 0;
		float yPosition_moving = 0;
	
		void stationary_callback(const turtlesim::msg::Pose::SharedPtr msg)
		{
			xPosition_stationary = msg->x;
			yPosition_stationary = msg->y;		
		}
		
		void moving_callback(const turtlesim::msg::Pose::SharedPtr msg)
		{
			xPosition_moving = msg->x;
			yPosition_moving = msg->y;			
		}
		
		void calculateDistance() 
		{
			double xPosition = abs(xPosition_stationary - xPosition_moving);
			double yPosition = abs(yPosition_stationary - yPosition_moving);
			
			double distance = sqrt((xPosition * xPosition) + (yPosition * yPosition));
			
			RCLCPP_INFO(this->get_logger(), "Position of X: %f Position of Y: %f", xPosition, yPosition);
			RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);

			publisher_->publish(xPosition);
			publisher_->publish(yPosition);
			publisher_->publish(distance);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_turtle_subscription_;
		rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_turtle_subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LocationSubscriber>());
	rclcpp::shutdown();
	return 0;
}

