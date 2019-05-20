cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_visualization)

add_compile_options(-std=c++14)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # suppress warning in Ogre
  add_compile_options(-Wno-deprecated-register)
endif()

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
#find_package(moveit_ros_robot_interaction REQUIRED)
#find_package(moveit_ros_warehouse REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(rviz2 REQUIRED) Must specify package
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(THIS_PACKAGE_INCLUDE_DIRS
  #motion_planning_rviz_plugin/include
  #planning_scene_rviz_plugin/include
  #robot_state_rviz_plugin/include
  rviz_plugin_render_tools/include
  #trajectory_rviz_plugin/include
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
)

set(THIS_PACKAGE_INCLUDE_LIBRARIES
  #moveit_motion_planning_rviz_plugin_core
  #moveit_planning_scene_rviz_plugin_core
  #moveit_robot_state_rviz_plugin_core
  moveit_rviz_plugin_render_tools
  #moveit_trajectory_rviz_plugin_core
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  #moveit_ros_planning_interface
  #moveit_ros_robot_interaction
  object_recognition_msgs
)

#[[install(
  PROGRAMS
  scripts/moveit_joy.py
  DESTINATION lib/${PROJECT_NAME}
)]]

add_subdirectory(rviz_plugin_render_tools)
#add_subdirectory(robot_state_rviz_plugin)
#add_subdirectory(planning_scene_rviz_plugin)
#add_subdirectory(motion_planning_rviz_plugin)
#add_subdirectory(trajectory_rviz_plugin)

install(FILES
  #motion_planning_rviz_plugin_description.xml
  #trajectory_rviz_plugin_description.xml
  #planning_scene_rviz_plugin_description.xml
  #robot_state_rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

#[[if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/moveit_joy.test)
endif()]]

ament_export_include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
ament_export_libraries(${THIS_PACKAGE_INCLUDE_LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
