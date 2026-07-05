cmake_minimum_required(VERSION 3.5)
project(ag_ros_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(rosodl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
#find_package(tf2_geometry REQUIRED)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
include_directories(include ${autopilot_msgs_INCLUDE_DIRS}
                            ${serial_INCLUDE_DIRS})
include_directories("/opt/ros/eloquent/include")#eloquent
include_directories(./src/AgLog)
include_directories(./src/AgProtocol)
include_directories(./src/AgRosMsgPub)
include_directories(./src/AgRosNode)
include_directories(./src/AgUtility)

aux_source_directory(./src SOURCES)
aux_source_directory(./src/AgLog SOURCES)
aux_source_directory(./src/AgProtocol SOURCES)
aux_source_directory(./src/AgRosMsgPub SOURCES)
aux_source_directory(./src/AgRosNode SOURCES)
aux_source_directory(./src/AgUtility SOURCES) 
add_executable(ag_ros_node ${SOURCES})

ament_target_dependencies(ag_ros_node rclcpp serial sensor_msgs tf2_ros tf2_msgs nav_msgs geometry_msgs)

install(TARGETS
  ag_ros_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
