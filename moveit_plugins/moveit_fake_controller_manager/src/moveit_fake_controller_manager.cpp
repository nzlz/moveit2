/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

/* Author: Ioan Sucan, Dave Coleman, Robert Haschke */

#include "moveit_fake_controllers.h"
#include "rclcpp/rclcpp.hpp"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_list_macros.hpp>
#include <map>
#include <iterator>

namespace moveit_fake_controller_manager
{
static const std::string DEFAULT_TYPE = "interpolate";
static const std::string ROBOT_DESCRIPTION = "robot_description";
static rclcpp::Logger LOGGER_FAKE_CONTROLLER_MANAGER = rclcpp::get_logger("moveit_fake_controller_manager").get_child("MoveItFakeControllerManager");
static rclcpp::Logger LOGGER_INITIAL_JOINT_VALUES = rclcpp::get_logger("moveit_fake_controller_manager").get_child("loadInitialJointValues");
class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItFakeControllerManager() : node_(rclcpp::Node::make_shared("fake_controller"))
  {
    if (!node_.hasParam("controller_list"))
    {
      RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER, "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER, "controller_list should be specified as an array");
      return;
    }

    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    bool latch = true;
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("fake_controller_joint_states", 100);
    /* publish initial pose */
    XmlRpc::XmlRpcValue initial;
    if (node_.getParam("initial", initial))
    {
      sensor_msgs::msg::JointState js = loadInitialJointValues(initial);
      js.header.stamp = rclcpp::Clock().now();
      pub_->publish(js);
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER, "Name and joints must be specified for each controller");
        continue;
      }

      try
      {
        const std::string name = std::string(controller_list[i]["name"]);

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER, "The list of joints for controller %s is not specified as an array",name.c_str());
          continue;
        }
        std::vector<std::string> joints;
        joints.reserve(controller_list[i]["joints"].size());
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          joints.emplace_back(std::string(controller_list[i]["joints"][j]));

        const std::string& type =
            controller_list[i].hasMember("type") ? std::string(controller_list[i]["type"]) : DEFAULT_TYPE;
        if (type == "last point")
          controllers_[name].reset(new LastPointController(name, joints, pub_));
        else if (type == "via points")
          controllers_[name].reset(new ViaPointController(name, joints, pub_));
        else if (type == "interpolate")
          controllers_[name].reset(new InterpolatingController(name, joints, pub_));
        else
          RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER,"Unknown fake controller type: %s", type.c_str());
      }
      catch (...)
      {
        RCLCPP_ERROR(LOGGER_FAKE_CONTROLLER_MANAGER, "Caught unknown exception while parsing controller information");
      }
    }
  }

  // TODO(anasarrak)
  // sensor_msgs::msg::JointState loadInitialJointValues(XmlRpc::XmlRpcValue& param) const
  // {
  //   sensor_msgs::msg::JointState js;
  //
  //   if (param.getType() != XmlRpc::XmlRpcValue::TypeArray || param.size() == 0)
  //   {
  //     RCLCPP_ERROR_ONCE(LOGGER_INITIAL_JOINT_VALUES, "Parameter 'initial' should be an array of (group, pose) "
  //                                                    "structs.");
  //     return js;
  //   }
  //
  //   robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  //   const robot_model::RobotModelPtr& robot_model = robot_model_loader.getModel();
  //   typedef std::map<std::string, double> JointPoseMap;
  //   JointPoseMap joints;
  //
  //   for (int i = 0, end = param.size(); i != end; ++i)
  //   {
  //     try
  //     {
  //       std::string group_name = std::string(param[i]["group"]);
  //       std::string pose_name = std::string(param[i]["pose"]);
  //       if (!robot_model->hasJointModelGroup(group_name))
  //       {
  //         ROS_WARN(LOGGER_INITIAL_JOINT_VALUES, "Unknown joint model group: %s", group_name);
  //         continue;
  //       }
  //       moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  //       moveit::core::RobotState robot_state(robot_model);
  //       const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
  //
  //       if (!robot_state.setToDefaultValues(jmg, pose_name))
  //       {
  //         RCLCPP_WARN(LOGGER_INITIAL_JOINT_VALUES, "Unknown pose '%s' for group '%s'.", pose_name.c_str(),
  //                        group_name.c_str());
  //         continue;
  //       }
  //       RCLCPP_INFO(LOGGER_INITIAL_JOINT_VALUES, "Set joints of group '%s' to pose '%s'.", group_name.c_str(),
  //                      pose_name.c_str());
  //
  //       for (std::vector<std::string>::const_iterator jit = joint_names.begin(), end = joint_names.end(); jit != end;
  //            ++jit)
  //       {
  //         const moveit::core::JointModel* jm = robot_state.getJointModel(*jit);
  //         if (!jm)
  //         {
  //           ROS_WARN_STREAM_NAMED(LOGGER_INITIAL_JOINT_VALUES, "Unknown joint: " << *jit);
  //           continue;
  //         }
  //         if (jm->getVariableCount() != 1)
  //         {
  //           ROS_WARN_STREAM_NAMED(LOGGER_INITIAL_JOINT_VALUES, "Cannot handle multi-variable joint: " << *jit);
  //           continue;
  //         }
  //
  //         joints[*jit] = robot_state.getJointPositions(jm)[0];
  //       }
  //     }
  //     catch (...)
  //     {
  //       RCLCPP_ERROR_ONCE(LOGGER_INITIAL_JOINT_VALUES, "Caught unknown exception while reading initial pose "
  //                                                      "information.");
  //     }
  //   }
  //
  //   // fill the joint state
  //   for (JointPoseMap::const_iterator it = joints.begin(), end = joints.end(); it != end; ++it)
  //   {
  //     js.name.push_back(it->first);
  //     js.position.push_back(it->second);
  //   }
  //   return js;
  // }

  ~MoveItFakeControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      RCLCPP_FATAL(LOGGER_FAKE_CONTROLLER_MANAGER,"No such controller: %s", name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    RCLCPP_INFO(LOGGER_FAKE_CONTROLLER_MANAGER,"Returned %d controllers in list", names.size());
  }

  /*
   * Fake controllers are always active
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Fake controllers are always loaded
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      RCLCPP_WARN(LOGGER_FAKE_CONTROLLER_MANAGER,"The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
               "the param server?",
               name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    return false;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::map<std::string, BaseFakeControllerPtr> controllers_;
};

}  // end namespace moveit_fake_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
