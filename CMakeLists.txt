cmake_minimum_required(VERSION 3.8)
project(image_centering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(epuck_driver_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(${PROJECT_NAME}_node src/image_centering.cpp src/pid.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)

# Installing node
install(TARGETS
${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME}
)

# Installing config files
# install(FILES
# 	config/safety_state_machine.yml
#   	DESTINATION share/${PROJECT_NAME}/config/
# )

# Installing launch files
# install(DIRECTORY launch
# DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_node 
rclcpp
epuck_driver_interfaces 
std_msgs
std_srvs
)

ament_package()
