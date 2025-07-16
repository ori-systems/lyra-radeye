cmake_minimum_required(VERSION 3.8)
project(radeye)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(radeye_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

add_executable(radeye_node src/radeye_node.cpp)
ament_target_dependencies(radeye_node rclcpp std_msgs radeye_msgs)

add_executable(radeye_daisychain_node src/radeye_daisychain_node.cpp)
ament_target_dependencies(radeye_daisychain_node rclcpp std_msgs radeye_msgs)


#############
## Install ##
#############

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  radeye_node
  radeye_daisychain_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#     DIRECTORY include/
#     DESTINATION include
# )

install(
    PROGRAMS
    scripts/radeye_radiation_point_visualiser.py
    scripts/random_tf.py
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()