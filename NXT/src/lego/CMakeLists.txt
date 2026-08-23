cmake_minimum_required(VERSION 3.8)
project(lego)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(stm32_serial_node src/test/stm32_serial_node.cpp)

install(FILES
  src/homografia.npy
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  stm32_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/test/stm32_comando_pub.py
  src/stm32_moveit_bridge.py
  src/test/stm32_moveit_bridge_.py
  src/test/save_pose.py
  src/save_pose_full.py
  src/test/play_sequence.py
  src/play_sequence_full.py
  src/test/plot_joint2_debug.py
  src/controladorCartesiano.py
  src/test/generador_cartesiano_real.py
  src/test/ida_y_vuelta_cartesiano.py
  src/presentacion/play_sequence_v2.py
  src/vision_aruco_node.py
  src/test/test_subscriber_node.py
  src/play_sequence_vision_completo.py 
  src/lego_rl_bridge.py
  src/lego_rl_bridge_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(stm32_serial_node
  rclcpp
  std_msgs
)

target_link_libraries(stm32_serial_node ${Boost_LIBRARIES})

ament_package()


