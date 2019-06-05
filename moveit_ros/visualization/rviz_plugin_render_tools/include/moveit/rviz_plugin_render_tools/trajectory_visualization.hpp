/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

#ifndef MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TRAJECTORY_VISUALIZATION
#define MOVEIT_TRAJECTORY_RVIZ_PLUGIN__TRAJECTORY_VISUALIZATION

#include <boost/thread/mutex.hpp>
#include <moveit/macros/class_forward.h>

#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>

#ifndef Q_MOC_RUN
#include "moveit/rviz_plugin_render_tools/robot_state_visualization.hpp"
#include "moveit/rviz_plugin_render_tools/trajectory_panel.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#endif

namespace rviz
{
class Robot;
class Shape;
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class ColorProperty;
class MovableText;
}

namespace moveit_rviz_plugin
{
MOVEIT_CLASS_FORWARD(TrajectoryVisualization);

class TrajectoryVisualization : public QObject
{
  Q_OBJECT

public:
  /**
   * \brief Playback a trajectory from a planned path
   * \param widget - either a rviz_common::Display or rviz_common::properties::Property
   * \param display - the rviz_common::Display from the parent
   * \return true on success
   */
  TrajectoryVisualization(rviz_common::properties::Property* widget, rviz_common::Display* display);

  ~TrajectoryVisualization() override;

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  void onInitialize(Ogre::SceneNode* scene_node, rviz_common::DisplayContext* context, const rclcpp::Node::SharedPtr ros_node);
  void onRobotModelLoaded(const robot_model::RobotModelConstPtr& robot_model);
  void onEnable();
  void onDisable();
  void setName(const QString& name);

  void dropTrajectory();

public Q_SLOTS:
  void interruptCurrentDisplay();
  void setDefaultAttachedObjectColor(const QColor& color);

private Q_SLOTS:

  /**
   * \brief Slot Event Functions
   */
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedLoopDisplay();
  void changedShowTrail();
  void changedTrailStepSize();
  void changedTrajectoryTopic();
  void changedStateDisplayTime();
  void changedRobotColor();
  void enabledRobotColor();
  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
  float getStateDisplayTime();
  void clearTrajectoryTrail();

  // Handles actually drawing the robot along motion plans
  RobotStateVisualizationPtr display_path_robot_;

  // Handle colouring of robot
  void setRobotColor(rviz_default_plugins::robot::Robot* robot, const QColor& color);
  void unsetRobotColor(rviz_default_plugins::robot::Robot* robot);

  robot_trajectory::RobotTrajectoryPtr displaying_trajectory_message_;
  robot_trajectory::RobotTrajectoryPtr trajectory_message_to_display_;
  std::vector<rviz_default_plugins::robot::Robot*> trajectory_trail_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_topic_sub_;

  bool animating_path_;
  bool drop_displaying_trajectory_;
  int current_state_;
  float current_state_time_;
  boost::mutex update_trajectory_message_;

  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  // Pointers from parent display taht we save
  rviz_common::Display* display_;  // the parent display that this class populates
  rviz_common::properties::Property* widget_;
  Ogre::SceneNode* scene_node_;
  rviz_common::DisplayContext* context_;
  rclcpp::Node::SharedPtr ros_node_;
  TrajectoryPanel* trajectory_slider_panel_;
  rviz_common::PanelDockWidget* trajectory_slider_dock_panel_;

  // Properties
  rviz_common::properties::BoolProperty* display_path_visual_enabled_property_;
  rviz_common::properties::BoolProperty* display_path_collision_enabled_property_;
  rviz_common::properties::EditableEnumProperty* state_display_time_property_;
  rviz_common::properties::RosTopicProperty* trajectory_topic_property_;
  rviz_common::properties::FloatProperty* robot_path_alpha_property_;
  rviz_common::properties::BoolProperty* loop_display_property_;
  rviz_common::properties::BoolProperty* trail_display_property_;
  rviz_common::properties::BoolProperty* interrupt_display_property_;
  rviz_common::properties::ColorProperty* robot_color_property_;
  rviz_common::properties::BoolProperty* enable_robot_color_property_;
  rviz_common::properties::IntProperty* trail_step_size_property_;
};

}  // namespace moveit_rviz_plugin

#endif
