/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI, Inc.
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

/* Author: David Hershberger */

#include "clear_octomap_service_capability.h"
#include <moveit/move_group/capability_names.h>

move_group::ClearOctomapService::ClearOctomapService() : MoveGroupCapability("ClearOctomapService")
{
}

void move_group::ClearOctomapService::initialize(std::shared_ptr<rclcpp::Node>& node)
{
  this->node_ = node;
  service_ = node_->create_service<std_srvs::srv::Empty>(
      CLEAR_OCTOMAP_SERVICE_NAME, std::bind(&ClearOctomapService::clearOctomap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) );
}
void move_group::ClearOctomapService::clearOctomap(const std::shared_ptr<rmw_request_id_t> request_header,
   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!context_->planning_scene_monitor_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ClearOctomapService"), "Cannot clear octomap since planning_scene_monitor_ does not exist.");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("ClearOctomapService"), "Clearing octomap...");
  context_->planning_scene_monitor_->clearOctomap();
  RCLCPP_INFO(rclcpp::get_logger("ClearOctomapService"), "Octomap cleared.");
  return;
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::ClearOctomapService, move_group::MoveGroupCapability)
