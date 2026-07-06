cmake_minimum_required(VERSION 3.8)
project(real_runner)

# ===================================================
# 0. 基础配置
# ===================================================
set(CALIBRATE off)
if(CALIBRATE)
  add_definitions(-DCALIBRATE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
# 1. 查找依赖
# ===================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

link_directories(lib)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS libUnitreeMotorSDK_M80106_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_M80106_Linux64.so)
endif()

# ===================================================
# 2. 创建静态库：包含共享的安全状态实现
# ===================================================
add_library(real_runner_utils_lib STATIC
    src/utils/secure_protect.cpp
)
# 确保库知道头文件路径
target_include_directories(real_runner_utils_lib PUBLIC include/real_runner)

# 静态库依赖
ament_target_dependencies(real_runner_utils_lib 
  rclcpp
  robot_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)


# ===================================================
# 3. 编译 real_runner 可执行文件 (主程序)
# ===================================================

add_executable(real_runner
  src/real_runner.cpp
  src/utils/set_zero.cpp
  ../../midware/src/vqf.cpp
)

# 链接库
target_link_libraries(real_runner 
    ${EXTRA_LIBS} 
    tf2::tf2 
    tf2_geometry_msgs::tf2_geometry_msgs
    real_runner_utils_lib
)

# 包含路径
target_include_directories(real_runner PUBLIC
    include/real_runner
    ../../midware/include
)

# 添加所有 ROS 依赖
ament_target_dependencies(real_runner 
    rclcpp 
    sensor_msgs 
    robot_msgs 
    tf2 
    tf2_geometry_msgs
)

install(TARGETS
  real_runner
  DESTINATION lib/${PROJECT_NAME})


# ===================================================
# 4. 安装配置
# ===================================================

install(FILES
  lib/${EXTRA_LIBS}
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
