#include <rclcpp_thread_test/test_fixture_node.hpp>

#include <gtest/gtest.h>


static const std::size_t ORDER = 5;

namespace rclcpp_multicore_ci_test
{

TestFixtureNode::TestFixtureNode()
  : Node("test_node")
  , cb_group_me_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  , fib_client_(rclcpp_action::create_client<Fibonacci>(this, "/fibonacci", cb_group_me_))
{
  auto on_one_shot_timer =
    [this]() -> void {
      one_shot_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");

      auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    };
  one_shot_timer_ = create_wall_timer(std::chrono::seconds(0), on_one_shot_timer);
}

bool TestFixtureNode::testActionFeebackCount()
{ 
  std::this_thread::sleep_for(std::chrono::seconds(3));

  if(!fib_client_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Timed out waiting for action server to be available");
    return false;
  }

  auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  options.goal_response_callback = [](std::shared_future<GoalHandleFibonacci::SharedPtr> future) -> void {
    (void)future;
  };

  std::mutex lk;
  std::vector<Fibonacci::Feedback> feedback;
  std::promise<void> promise;

  options.feedback_callback = [=, &feedback, &lk,
                               &promise](GoalHandleFibonacci::SharedPtr gh,
                                         std::shared_ptr<const Fibonacci::Feedback> fb) -> void {
    (void)gh;
    std::scoped_lock _(lk);
    feedback.push_back(*fb);
    RCLCPP_INFO_STREAM(get_logger(), "Have " << feedback.size() << " state feedback messages");
    if (feedback.size() == ORDER - 1)
    {
      promise.set_value();
    }
  };

  options.result_callback = [](const GoalHandleFibonacci::WrappedResult& result) -> void { (void)result; };

  auto goal = Fibonacci::Goal();
  goal.order = ORDER;

  auto fut = fib_client_->async_send_goal(goal, options);

  if (fut.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Timed out waiting for goal response callback");
    return false;
  }

  auto gh = fut.get();
  auto res_fut = fib_client_->async_get_result(gh);
  if (res_fut.wait_for(std::chrono::seconds(ORDER + 1)) == std::future_status::timeout)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Timed out waiting for result callback");
    return false;
  }

  auto received_state_fut = promise.get_future();
  if (received_state_fut.wait_for(std::chrono::seconds(ORDER + 1)) == std::future_status::timeout)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Timed out waiting for feedback vector to be populated");
    return false;
  }

  if (feedback.size() != ORDER - 1)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Wrong number of feedback messages: expected " << ORDER - 1 <<  ", got " << feedback.size() << ".");
    return false;
  }

  return true;
}

}  // namespace rclcpp_multicore_ci_test

TEST(TestMulticoreCI, require_action_feedback)
{
  auto node = std::make_shared<rclcpp_multicore_ci_test::TestFixtureNode>();

  rclcpp::executors::MultiThreadedExecutor exec;

  auto exec_thread = std::make_unique<std::thread>([&exec, &node]() {
    exec.add_node(node);
    exec.spin();
    exec.remove_node(node);
  });

  ASSERT_TRUE(node->testActionFeebackCount());

  exec.cancel();
  exec_thread->join();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
