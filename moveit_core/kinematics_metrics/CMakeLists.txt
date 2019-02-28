set(MOVEIT_LIB_NAME moveit_kinematics_metrics)

add_library(${MOVEIT_LIB_NAME} src/kinematics_metrics.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_robot_state
  urdf
  urdfdom_headers)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)
