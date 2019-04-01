set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME} src/planning_scene_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_common_planning_interface_objects
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
  moveit_core)

#TODO(anasarrak): uncomment this and fix it for ROS2
# add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_planning_scene_interface.cpp)
# set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#
# ament_target_dependencies(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME
#   _moveit_planning_scene_interface PREFIX ""
#   moveit_core)


install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# install(TARGETS ${MOVEIT_LIB_NAME}_python
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
