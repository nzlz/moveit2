set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)

set(HEADERS
  include/moveit/rviz_plugin_render_tools/octomap_render.hpp
  include/moveit/rviz_plugin_render_tools/planning_link_updater.hpp
  include/moveit/rviz_plugin_render_tools/planning_scene_render.hpp
  include/moveit/rviz_plugin_render_tools/render_shapes.hpp
  include/moveit/rviz_plugin_render_tools/robot_state_visualization.hpp
  include/moveit/rviz_plugin_render_tools/trajectory_visualization.hpp
  include/moveit/rviz_plugin_render_tools/trajectory_panel.hpp
)

include_directories(include
)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/render_shapes.cpp
  src/robot_state_visualization.cpp
  src/planning_scene_render.cpp
  src/planning_link_updater.cpp
  src/octomap_render.cpp
  src/trajectory_visualization.cpp
  src/trajectory_panel.cpp
  ${HEADERS}
)
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
  rviz_default_plugins
  Boost
  rclcpp
  rviz_ogre_vendor
)

target_link_libraries(${MOVEIT_LIB_NAME}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION share/${MOVEIT_LIB_NAME})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION share/${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION share/${MOVEIT_LIB_NAME})
