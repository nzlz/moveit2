#if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # if(WIN32)
	#   # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
	# else()
	#   set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils")
	# endif()


  ament_add_gtest(test_kinematics_plugin  test_kinematics_plugin.cpp
				# APPEND_LIBRARY_DIRS "${append_library_dirs}"
	)


  target_include_directories(test_kinematics_plugin PUBLIC
    ${pluginlib_INCLUDE_DIRS}
  )

  target_link_libraries(test_kinematics_plugin
    ${pluginlib_LIBRARIES}
  )

  # set(DEPS DEPENDENCIES test_kinematics_plugin)
  # set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
  # add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
  # add_rostest(panda-kdl.test ${DEPS} ${ARGS})
  #
  # # LMA solver seems to have less precision
  # set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin tolerance:=0.0005)
  # add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
  # add_rostest(panda-kdl.test ${DEPS} ${ARGS})
  #
  # # Run ikfast tests only if the corresponding packages were built
  # find_package(fanuc_ikfast_plugin QUIET)
  # if (fanuc_ikfast_plugin_FOUND)
  #   add_rostest(fanuc-ikfast.test ${DEPS})
  # endif()
  #
  # find_package(panda_ikfast_plugin QUIET)
  # if (panda_ikfast_plugin_FOUND)
  #   add_rostest(panda-ikfast.test ${DEPS})
  # endif()
#endif()
