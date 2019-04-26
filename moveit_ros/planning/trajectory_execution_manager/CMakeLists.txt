set(MOVEIT_LIB_NAME moveit_trajectory_execution_manager)

add_library(${MOVEIT_LIB_NAME} SHARED src/trajectory_execution_manager.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_planning_scene_monitor moveit_robot_model_loader
  rclcpp
  Boost)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

# TODO (anasarrak): Fix the tests
# add_library(test_controller_manager_plugin test/test_moveit_controller_manager_plugin.cpp)
# set_target_properties(test_controller_manager_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# ament_target_dependencies(test_controller_manager_plugin
#   ${MOVEIT_LIB_NAME}
#   rclcpp
#   Boost)
#
# add_executable(test_controller_manager test/test_app.cpp)
# ament_target_dependencies(test_controller_manager
#   ${MOVEIT_LIB_NAME}
#   rclcpp
#   Boost)
