#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("trajectory_execution_manager_minimal_subscriber")
  {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 1;
    qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      [this](const std_msgs::msg::String::ConstSharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }, qos);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
//
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// using std::placeholders::_1;
//
// class MinimalSubscriber : public rclcpp::Node
// {
// public:
//   MinimalSubscriber()
//   : Node("minimal_subscriber")
//   {
//     rmw_qos_profile_t qos = rmw_qos_profile_default;
//     qos.depth = 1;
//     qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
//     // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
//     qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
//     qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
//     // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
//     subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "robot_description", std::bind(&MinimalSubscriber::topic_callback, this, _1), qos);
//   }
//
// private:
//   void topic_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//   }
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };
//
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
//
// rclcpp::Node::SharedPtr g_node = nullptr;
//
// /* We do not recommend this style anymore, because composition of multiple
//  * nodes in the same executable is not possible. Please see one of the subclass
//  * examples for the "new" recommended styles. This example is only included
//  * for completeness because it is similar to "classic" standalone ROS nodes. */
//
// void topic_callback(const std_msgs::msg::String::SharedPtr msg)
// {
//   RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg->data.c_str());
// }
//
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   g_node = rclcpp::Node::make_shared("minimal_subscriber");
//
//   printf("minimal_subscriber node\n");
//
//   rmw_qos_profile_t qos = rmw_qos_profile_default;
//   qos.depth = 1;
//   qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
//
//   auto subscription = g_node->create_subscription<std_msgs::msg::String>
//       ("/robot_description", topic_callback, qos);
//   printf("subscription to /robot_description\n");
//
//   rclcpp::spin(g_node);
//   rclcpp::shutdown();
//   // TODO(clalancette): It would be better to remove both of these nullptr
//   // assignments and let the destructors handle it, but we can't because of
//   // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
//   // we should probably look at removing these two assignments.
//   subscription = nullptr;
//   g_node = nullptr;
//   return 0;
// }
