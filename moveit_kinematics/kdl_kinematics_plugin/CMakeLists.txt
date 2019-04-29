set(MOVEIT_LIB_NAME moveit_kdl_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/kdl_kinematics_plugin.cpp
  src/chainiksolver_vel_mimic_svd.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_compile_definitions(moveit_kdl_kinematics_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(moveit_kdl_kinematics_plugin PRIVATE "TEST_PLUGINLIB_FIXTURE_BUILDING_LIBRARY")

target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${random_numbers_INCLUDE_DIRS}
)

target_link_libraries(${MOVEIT_LIB_NAME}
  ${rclcpp_LIBRARIES}
  ${random_numbers_LIBRARIES}
)

# install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
