cmake_minimum_required(VERSION 3.5)
project(moveit_simple_controller_manager)

add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/moveit_simple_controller_manager.cpp
  # src/follow_joint_trajectory_controller_handle.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  Boost
  moveit_core
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME}
)
target_include_directories(${PROJECT_NAME} PUBLIC
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION include/${PROJECT_NAME}
       )

ament_export_include_directories(include)
ament_export_dependencies(moveit_core)
ament_export_dependencies(control_msgs)
ament_package()
