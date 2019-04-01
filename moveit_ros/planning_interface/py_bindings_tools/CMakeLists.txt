set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME} src/roscpp_initializer.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  Boost
  python)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_roscpp_initializer.cpp)
ament_target_dependencies(${MOVEIT_LIB_NAME}_python
  ${MOVEIT_LIB_NAME}
  python
  rclcpp
  Boost)

set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")

install(TARGETS ${MOVEIT_LIB_NAME}_python
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
