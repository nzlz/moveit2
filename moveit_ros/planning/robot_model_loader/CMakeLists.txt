set(MOVEIT_LIB_NAME moveit_robot_model_loader)

if(WIN32)
  # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
else()
  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/../collision_plugin_loader;${CMAKE_CURRENT_BINARY_DIR}/../moveit_kinematics_plugin_loader;${CMAKE_CURRENT_BINARY_DIR}/../rdf_loader")
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(${MOVEIT_LIB_NAME} SHARED src/robot_model_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_rdf_loader
  moveit_kinematics_plugin_loader
  moveit_collision_plugin_loader
  ${moveit_ros_planning_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
