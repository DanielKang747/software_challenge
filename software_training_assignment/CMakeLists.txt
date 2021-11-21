cmake_minimum_required(VERSION 3.5)
project(software_training_assignment)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
	"action/Waypoint.action"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(action_server SHARED src/waypoint_action_server.cpp)

target_compile_definitions(action_server
	PRIVATE "SOFTWARE_TRAINING_BUILDING_DLL")
		
ament_target_dependencies(action_server
	rclcpp
	rclcpp_action
	rclcpp_components
	turtlesim
	std_msgs
	geometry_msgs)

rosidl_target_interfaces(action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(action_server PLUGIN "software_training_assignment::WaypointActionServer" EXECUTABLE waypoint_action_server)


ament_export_dependencies(rosidl_default_runtime)

add_executable(client src/ros2_spawn_client.cpp)
ament_target_dependencies(client rclcpp turtlesim)

add_executable(client1 src/ros2_clear_client.cpp)
ament_target_dependencies(client1 rclcpp turtlesim)

add_executable(client2 src/ros2_turn_subscriber.cpp)
ament_target_dependencies(client2 rclcpp geometry_msgs)

add_executable(client3 src/ros2_reset_turtle_service.cpp)
ament_target_dependencies(client3 rclcpp turtlesim)

add_executable(client4 src/ros2_message_publisher.cpp)
ament_target_dependencies(client4 rclcpp turtlesim)

install(TARGETS
client 
client1
client2
client3
client4
action_server
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/)

ament_package()
