set(MOVEIT_LIB_NAME moveit_plan_execution)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/plan_with_sensing.cpp
  src/plan_execution.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_planning_pipeline
  moveit_trajectory_execution_manager
  moveit_planning_scene_monitor
  rclcpp
  Boost
  )

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
