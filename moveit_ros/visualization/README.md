# MOVEIT2 PORT [WIP]

## Ported
 - robot_state_rviz_plugin [Not Tested]
 - rviz_plugin_render_tools [Not Tested]
 - planning_scene_rviz_plugin [Not Tested]

## Unported
 - Joy
 - motion_planning_rviz_plugin (read below)

 ---

 ### `motion_planning_rviz_plugin` unported dependencies

- moveit_ros robot_interaction

```cpp
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/robot_interaction/kinematic_options_map.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
```

- ros2 interactive_markers
New PR [WIP] at https://github.com/ros-visualization/interactive_markers/pull/41

```cpp
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
```
Rviz interactive_markers plugin
```cpp
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
```

- simple_action_client

Replace the code using `simple_action_client` by ros2 `rclcpp_action`. Probably helpful: _moveit2/.../action_based_controller_handle.h_.

- rviz_common display_factory.h

display_factory is accessible only locally, the following include should not work.
```cpp
#include <rviz_common/display_factory.h> 
```

- moveit_ros warehouse, depends on the unported warehouse_ros
```cpp
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
```

