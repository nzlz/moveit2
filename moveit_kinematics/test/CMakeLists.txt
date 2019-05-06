if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(WIN32)
	  # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
	else()
	  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../kdl_kinematics_plugin")
	endif()

  ament_add_gtest(test_kinematics_plugin  test_kinematics_plugin.cpp
				APPEND_LIBRARY_DIRS "${append_library_dirs}"
	)

  ament_target_dependencies(
    test_kinematics_plugin
    class_loader
    pluginlib
    rclcpp
    moveit_core
    moveit_ros_planning
  )
  target_compile_definitions(test_kinematics_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  target_link_libraries(test_kinematics_plugin
    moveit_kdl_kinematics_plugin
    ${orocos_kdl_LIBRARIES}
  )
endif()
