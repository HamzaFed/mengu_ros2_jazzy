cmake_minimum_required(VERSION 3.8)
project(mengu_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Qt
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# =========================
# Executables
# =========================
add_executable(mengu_gui_node src/mengu_gui_node.cpp)
add_executable(simple_gui src/simple_gui.cpp)

# =========================
# ROS dependencies
# =========================
ament_target_dependencies(
  mengu_gui_node
  rclcpp
  trajectory_msgs
)

ament_target_dependencies(
  simple_gui
  rclcpp
  trajectory_msgs
)

# =========================
# Qt link
# =========================
target_link_libraries(mengu_gui_node
  Qt5::Widgets
)

target_link_libraries(simple_gui
  Qt5::Widgets
)

# =========================
# Install
# =========================
install(TARGETS
  mengu_gui_node
  simple_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
