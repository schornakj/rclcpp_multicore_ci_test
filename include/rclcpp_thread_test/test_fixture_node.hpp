#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_tutorials_interfaces/action/fibonacci.hpp>

namespace rclcpp_multicore_ci_test
{
using action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class TestFixtureNode : public rclcpp::Node
{
public:
  TestFixtureNode();

  bool testActionFeebackCount();

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_me_;
  rclcpp_action::Client<Fibonacci>::SharedPtr fib_client_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
};
}  // namespace rclcpp_multicore_ci_test
