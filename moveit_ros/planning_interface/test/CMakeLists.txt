# if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)

  add_executable(test_cleanup
    test_cleanup.cpp)

  ament_target_dependencies(test_cleanup
    rclcpp
    Boost
    moveit_msgs
  )
  target_link_libraries(test_cleanup
    ${MOVEIT_LIB_NAME}
    moveit_common_planning_interface_objects
    moveit_move_group_interface
  )

  # add_rostest(python_move_group.test)
  # add_rostest(python_move_group_ns.test)
  # add_rostest(robot_state_update.test)
  # add_rostest(cleanup.test)
# endif()
