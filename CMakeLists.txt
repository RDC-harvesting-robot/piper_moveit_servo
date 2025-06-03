cmake_minimum_required(VERSION 3.5)
project(piper_moveit_servo)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)


include_directories(include)


add_executable(piper_servo_keyboard_input src/piper_servo_keyboard_input.cpp)
target_include_directories(piper_servo_keyboard_input PUBLIC include)
ament_target_dependencies(piper_servo_keyboard_input
  rclcpp
  control_msgs
  geometry_msgs
)


add_executable(piper_servo_cpp_interface_demo src/piper_servo_cpp_interface_demo.cpp)
target_include_directories(piper_servo_cpp_interface_demo PUBLIC include)
ament_target_dependencies(piper_servo_cpp_interface_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_servo
)

# ノード3: piper_servo_demo
#add_executable(piper_servo_demo src/piper_servo_demo.cpp)
#target_include_directories(piper_servo_demo PUBLIC include)
#ament_target_dependencies(piper_servo_demo
#  rclcpp
#  control_msgs
#  moveit_ros_planning_interface
#  moveit_servo
#)

# 実行ファイルのインストール
install(
  TARGETS
    piper_servo_keyboard_input
    piper_servo_cpp_interface_demo
#    piper_servo_demo
  DESTINATION
    lib/${PROJECT_NAME}
)

# launch フォルダのインストール
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# config フォルダのインストール
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 最後に必要
ament_package()
