set(MOVEIT_LIB_NAME moveit_kinematics_plugin_loader)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_plugin_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_rdf_loader
  ${srdfdom_LIBRARIES}
  ${moveit_core_LIBRARIES}
)


install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
