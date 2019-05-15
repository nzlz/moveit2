cmake_minimum_required(VERSION 3.5)
project(moveit_planners_ompl)

# At least C++11 required for OMPL
add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

# generate_dynamic_reconfigure_options("ompl_interface/cfg/OMPLDynamicReconfigure.cfg")

include_directories(ompl_interface/include
                    ${Boost_INCLUDE_DIRS}
                    ${rclcpp_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS}
                    ${srdfdom_INCLUDE_DIRS}
)

add_subdirectory(ompl_interface)

pluginlib_export_plugin_description_file(moveit_core ompl_interface_plugin_description.xml)

ament_export_include_directories(include)
ament_export_dependencies(moveit_core)
ament_export_dependencies(ompl)
ament_package()
