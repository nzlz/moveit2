/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/constraint_samplers/constraint_sampler_tools.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

rclcpp::Logger LOGGER_CONTRAINT_SAMPLERS_TOOLS = rclcpp::get_logger("moveit").get_child("constraint_samplers");

void constraint_samplers::visualizeDistribution(const moveit_msgs::msg::Constraints& constr,
                                                const planning_scene::PlanningSceneConstPtr& scene,
                                                const std::string& group, const std::string& link_name,
                                                unsigned int sample_count, visualization_msgs::msg::MarkerArray& markers)
{
  visualizeDistribution(ConstraintSamplerManager::selectDefaultSampler(scene, group, constr), scene->getCurrentState(),
                        link_name, sample_count, markers);
}

double constraint_samplers::countSamplesPerSecond(const moveit_msgs::msg::Constraints& constr,
                                                  const planning_scene::PlanningSceneConstPtr& scene,
                                                  const std::string& group)
{
  return countSamplesPerSecond(ConstraintSamplerManager::selectDefaultSampler(scene, group, constr),
                               scene->getCurrentState());
}

double constraint_samplers::countSamplesPerSecond(const ConstraintSamplerPtr& sampler,
                                                  const robot_state::RobotState& reference_state)
{
  if (!sampler)
  {
    RCLCPP_ERROR(LOGGER_CONTRAINT_SAMPLERS_TOOLS, "No sampler specified for counting samples per second");
    return 0.0;
  }
  robot_state::RobotState ks(reference_state);
  unsigned long int valid = 0;
  unsigned long int total = 0;
  //TODO RCLCPP::WallTimer has not a .now() function, and there is no wallDuration (they use std::chrono::nanoseconds)
  // I'll use rclcpp::time, but this is something to fix.
  rclcpp::Duration duration(1,0);
  rclcpp::Time end = rclcpp::Clock().now() + duration;
  // ros::WallTime end = ros::WallTime::now() + ros::WallDuration(1.0);
  do
  {
    static const unsigned int N = 10;
    total += N;
    for (unsigned int i = 0; i < N; ++i)
    {
      if (sampler->sample(ks, 1))
        valid++;
    }
  } while (rclcpp::Clock().now() < end);
  return (double)valid / (double)total;
}

void constraint_samplers::visualizeDistribution(const ConstraintSamplerPtr& sampler,
                                                const robot_state::RobotState& reference_state,
                                                const std::string& link_name, unsigned int sample_count,
                                                visualization_msgs::msg::MarkerArray& markers)
{
  if (!sampler)
  {
    RCLCPP_ERROR(LOGGER_CONTRAINT_SAMPLERS_TOOLS, "No sampler specified for visualizing distribution of samples");
    return;
  }
  const robot_state::LinkModel* lm = reference_state.getLinkModel(link_name);
  if (!lm)
    return;
  robot_state::RobotState ks(reference_state);
  rclcpp::Time stamp = rclcpp::Clock().now();
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0f;
  color.g = 0.0f;
  color.b = 0.0f;
  color.a = 1.0f;
  for (unsigned int i = 0; i < sample_count; ++i)
  {
    if (!sampler->sample(ks))
      continue;
    const Eigen::Vector3d& pos = ks.getGlobalLinkTransform(lm).translation();
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = stamp;
    mk.header.frame_id = sampler->getJointModelGroup()->getParentModel().getModelFrame();
    mk.ns = "constraint_samples";
    mk.id = i;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = pos.x();
    mk.pose.position.y = pos.y();
    mk.pose.position.z = pos.z();
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
    mk.color = color;
    mk.lifetime = rclcpp::Duration(30.0);
    markers.markers.push_back(mk);
  }
}
