#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "software_training_assignment/action/waypoint.hpp"

#include "software_training_assignment/visibility.h"

namespace software_training_assignment
{
class WaypointActionServer : public rclcpp::Node
{
public:
  using Waypoint = software_training_assignment::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
  explicit WaypointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_action_server", options)
  {
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    moving_turt_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/moving_turtle/cmd_vel", 10);

    moving_turt_subscriber = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", 10, std::bind(&WaypointActionServer::subscriber_callback, this, _1));

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
      this,
      "Waypoint",
      std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
      std::bind(&WaypointActionServer::handle_cancel, this, _1),
      std::bind(&WaypointActionServer::handle_accepted, this, _1));
  }

private:

  float moving_turt_x = 0;
  float moving_turt_y = 0;
  float moving_turt_theta = 0;
  float moving_turt_linear_vel = 0;
  float moving_turt_angular_vel = 0;

  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Waypoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal");
    RCLCPP_INFO(this->get_logger(), "linear X: %f Y: %f Z:%f", goal->linear_pos.x, goal->linear_pos.y, goal->linear_pos.z);
    RCLCPP_INFO(this->get_logger(), "angular X: %f Y:%f Z: %f", goal->angular_pos.x, goal->angular_pos.y, goal->angular_pos.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
	  moving_turt_x = msg->x;
	  moving_turt_y = msg->y;
	  moving_turt_theta = msg->theta;
	  moving_turt_linear_vel = msg->linear_velocity;
	  moving_turt_angular_vel = msg->angular_velocity;

  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WaypointActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Time start_time = this->now();

    rclcpp::Rate cycle_rate(1);
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<Waypoint::Feedback>();

    auto result = std::make_shared<Waypoint::Result>();

    float &curr_x = feedback->x_pos;
    float &curr_y = feedback->y_pos;
    float &curr_theta = feedback->theta;

    float lin_x{0};
    float lin_y{0};
    float lin_z{0};

    float ang_x{0};
    float ang_y{0};
    float ang_z{0};

    while (rclcpp::ok() && (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y || lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x || ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z))
    {
	    if (goal_handle->is_canceling()) {
		    RCLCPP_INFO(this->get_logger(), "Goal Canceled");
		    
		    rclcpp::Time curr_time = this->now();
		    rclcpp::Duration time = curr_time - start_time;
		    long int duration{time.nanoseconds()};
		    result->duration = duration;

		    goal_handle->canceled(std::move(result));
		    return;
	    }

	    auto message_cmd_vel = geometry_msgs::msg::Twist();

	    message_cmd_vel.linear.x = (lin_x < goal->linear_pos.x) ? lin_x++ : lin_x;
	    message_cmd_vel.linear.y = (lin_y < goal->linear_pos.y) ? lin_y++ : lin_y;
	    message_cmd_vel.linear.z = (lin_z < goal->linear_pos.z) ? lin_z++ : lin_z;
	    message_cmd_vel.angular.x = (ang_x < goal->angular_pos.x) ? ang_x++: ang_x;
	    message_cmd_vel.angular.y = (ang_y < goal->angular_pos.y) ? ang_y++: ang_y;
	    message_cmd_vel.angular.z = (ang_z < goal->angular_pos.z) ? ang_z++: ang_z;

	    moving_turt_publisher->publish(std::move(message_cmd_vel));

	    curr_x = moving_turt_x - lin_x;
	    curr_y = moving_turt_y - lin_y;

	    float theta{0};

	    {
		    float x1{lin_x}, x2{lin_y}, x3{lin_z};

		    float magnitude = sqrt((x1 * x1) + (x2 * x2) + (x3 * x3));

		    theta = acos(x3 / magnitude);
	    }

	    curr_theta = moving_turt_theta - theta;

	    goal_handle->publish_feedback(std::move(feedback));

	    cycle_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
	    rclcpp::Time end = this->now();
	    rclcpp::Duration duration = end - start_time;
	    long int res_time(duration.nanoseconds());

	    result->duration = res_time;

      goal_handle->succeed(std::move(result));
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr moving_turt_publisher;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_turt_subscriber;

};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(software_training_assignment::WaypointActionServer)
