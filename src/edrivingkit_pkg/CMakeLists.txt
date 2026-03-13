cmake_minimum_required(VERSION 3.8)
project(edrivingkit_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCANBASIC_INCLUDE_DIRS /usr/inlclude)
set(PCAN_LIBRARY /usr/lib/libpcanbasic.so)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PCANBASIC_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/KitCmdMsg.msg"
  "msg/KitFeedMsg.msg"
  # DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target 
                      "${PROJECT_NAME}" 
                      "rosidl_typesupport_cpp"
)

add_executable(edrivingkit_pcan_node src/edrivingkit_pcan_node.cpp)
ament_target_dependencies(edrivingkit_pcan_node rclcpp )
target_link_libraries(edrivingkit_pcan_node  "${cpp_typesupport_target}" ${PCAN_LIBRARY})

add_executable(edrivingkit_for_vcu_node src/edrivingkit_for_vcu_node.cpp)
ament_target_dependencies(edrivingkit_for_vcu_node rclcpp )
target_link_libraries(edrivingkit_for_vcu_node  "${cpp_typesupport_target}" ${PCAN_LIBRARY})


install(TARGETS 
  edrivingkit_pcan_node
  edrivingkit_for_vcu_node

  # install the executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/Main_Prog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
