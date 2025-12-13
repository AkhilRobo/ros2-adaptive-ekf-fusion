cmake_minimum_required(VERSION 3.8)
project(antarctic_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)


install(DIRECTORY
  launch
  urdf
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
