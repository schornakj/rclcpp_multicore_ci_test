# rclcpp_multicore_ci_test

## What Is This?

I've been having issues running integration tests in CI that rely on getting action feedback from a server. It looks like there's some problem related to only having a few CPU cores available in CI that may be exposing a bug in rclcpp. This package provides a minimally-reproducible test case that can be run locally.

## Steps to Reproduce

Assuming you've cloned this repo into some location like `~/workspaces/test_ws`, and the current working directory is `~/workspaces/test_ws`, run the commands listed below.

My dev workstation is a Lenovo Thinkpad P15 with an Intel i7-10750H CPU, which has 12 cores, and 16 GB RAM.

Tested on ROS2 Foxy and Rolling.

### Expect to Succeed

This usually succeeds:

```
python3 -u /opt/ros/rolling/share/ament_cmake_test/cmake/run_test.py /tmp/asd \
  --package-name rclcpp_multicore_ci_test \
  --env TEST_LAUNCH_DIR=src/rclcpp_multicore_ci_test/test TEST_EXECUTABLE=build/rclcpp_multicore_ci_test/test_fixture_node \
  --command src/rclcpp_multicore_ci_test/test/test_nodes.py
```

Successful output looks like this for me:

```
-- run_test.py: extra environment variables:
 - TEST_EXECUTABLE=build/rclcpp_multicore_ci_test/test_fixture_node
 - TEST_LAUNCH_DIR=src/rclcpp_multicore_ci_test/test
-- run_test.py: invoking following command in '/home/jschornak/workspaces/test_ws':
 - src/rclcpp_multicore_ci_test/test/test_nodes.py
[INFO] [launch]: All log files can be found below /home/jschornak/.ros/log/2021-05-19-13-33-17-001454-jschornak-PickNik-ThinkPad-P15-2053470
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [fibonacci_action_server-1]: process started with pid [2053472]
[INFO] [test_node-2]: process started with pid [2053474]
[test_node-2] [==========] Running 1 test from 1 test suite.
[test_node-2] [----------] Global test environment set-up.
[test_node-2] [----------] 1 test from TaskSequenceInterface
[test_node-2] [ RUN      ] TaskSequenceInterface.setup_plan_exec_through_ros
[fibonacci_action_server-1] 1621445597.070719 [0] fibonacci_: using network interface wlp0s20f3 (udp/192.168.0.4) selected arbitrarily from: wlp0s20f3, docker0
[test_node-2] 1621445597.071207 [0] test_fixtu: using network interface wlp0s20f3 (udp/192.168.0.4) selected arbitrarily from: wlp0s20f3, docker0
[fibonacci_action_server-1] [INFO] [1621445597.075823790] [fibonacci_action_server]: Received goal request with order 5
[fibonacci_action_server-1] [INFO] [1621445597.076032682] [fibonacci_action_server]: Executing goal
[fibonacci_action_server-1] [INFO] [1621445597.076107494] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445597.076194975] [test_node]: Have 1 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445598.076383943] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445598.076632686] [test_node]: Have 2 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445599.076409964] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445599.076681886] [test_node]: Have 3 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445600.076326035] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445600.076586931] [test_node]: Have 4 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445601.076644147] [fibonacci_action_server]: Goal succeeded
[test_node-2] [       OK ] TaskSequenceInterface.setup_plan_exec_through_ros (4216 ms)
[test_node-2] [----------] 1 test from TaskSequenceInterface (4216 ms total)
[test_node-2] 
[test_node-2] [----------] Global test environment tear-down
[test_node-2] [==========] 1 test from 1 test suite ran. (4217 ms total)
[test_node-2] [  PASSED  ] 1 test.
[INFO] [test_node-2]: process has finished cleanly [pid 2053474]
[INFO] [fibonacci_action_server-1]: sending signal 'SIGINT' to process[fibonacci_action_server-1]
[fibonacci_action_server-1] [INFO] [1621445601.290434292] [rclcpp]: signal_handler(signal_value=2)
[INFO] [fibonacci_action_server-1]: process has finished cleanly [pid 2053472]
-- run_test.py: return code 0
-- run_test.py: generate result file '/tmp/asd' with failed test
-- run_test.py: verify result file '/tmp/asd'

```

### Should Succeed but Sometimes Fails

Prepend `taskset -c 0-1` to the command to force the tests to only run on 2 cores, which mimics the available hardware on many CI platforms like Github Actions.

```
taskset -c 0-1 python3 -u /opt/ros/rolling/share/ament_cmake_test/cmake/run_test.py /tmp/asd \
  --package-name rclcpp_multicore_ci_test \
  --env TEST_LAUNCH_DIR=src/rclcpp_multicore_ci_test/test TEST_EXECUTABLE=build/rclcpp_multicore_ci_test/test_fixture_node \
  --command src/rclcpp_multicore_ci_test/test/test_nodes.py
```

This fails about 10% of the time for me.

It looks like one of the expected feedback messages is published but doesn't trigger the feedback callback in the action client.

```
-- run_test.py: extra environment variables:
 - TEST_EXECUTABLE=build/rclcpp_multicore_ci_test/test_fixture_node
 - TEST_LAUNCH_DIR=src/rclcpp_multicore_ci_test/test
-- run_test.py: invoking following command in '/home/jschornak/workspaces/test_ws':
 - src/rclcpp_multicore_ci_test/test/test_nodes.py
[INFO] [launch]: All log files can be found below /home/jschornak/.ros/log/2021-05-19-13-34-28-808936-jschornak-PickNik-ThinkPad-P15-2053671
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [fibonacci_action_server-1]: process started with pid [2053673]
[INFO] [test_node-2]: process started with pid [2053675]
[test_node-2] [==========] Running 1 test from 1 test suite.
[test_node-2] [----------] Global test environment set-up.
[test_node-2] [----------] 1 test from TaskSequenceInterface
[test_node-2] [ RUN      ] TaskSequenceInterface.setup_plan_exec_through_ros
[fibonacci_action_server-1] 1621445668.874821 [0] fibonacci_: using network interface wlp0s20f3 (udp/192.168.0.4) selected arbitrarily from: wlp0s20f3, docker0
[test_node-2] 1621445668.875303 [0] test_fixtu: using network interface wlp0s20f3 (udp/192.168.0.4) selected arbitrarily from: wlp0s20f3, docker0
[fibonacci_action_server-1] [INFO] [1621445668.891735102] [fibonacci_action_server]: Received goal request with order 5
[fibonacci_action_server-1] [INFO] [1621445668.892005547] [fibonacci_action_server]: Executing goal
[fibonacci_action_server-1] [INFO] [1621445668.892090501] [fibonacci_action_server]: Publish feedback
[fibonacci_action_server-1] [INFO] [1621445669.892521092] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445669.892611049] [test_node]: Have 1 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445670.892371325] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445670.892701509] [test_node]: Have 2 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445671.892378535] [fibonacci_action_server]: Publish feedback
[test_node-2] [INFO] [1621445671.892631492] [test_node]: Have 3 state feedback messages
[fibonacci_action_server-1] [INFO] [1621445672.892589940] [fibonacci_action_server]: Goal succeeded
[test_node-2] [ERROR] [1621445678.893078911] [test_node]: Timed out waiting for feedback vector to be populated
[test_node-2] terminate called without an active exception
[test_node-2] /home/jschornak/workspaces/test_ws/src/rclcpp_multicore_ci_test/src/test_fixture_node.cpp:99: Failure
[test_node-2] Value of: node->testActionFeebackCount()
[test_node-2]   Actual: false
[test_node-2] Expected: true
[ERROR] [test_node-2]: process has died [pid 2053675, exit code -6, cmd 'build/rclcpp_multicore_ci_test/test_fixture_node'].
[INFO] [fibonacci_action_server-1]: sending signal 'SIGINT' to process[fibonacci_action_server-1]
[fibonacci_action_server-1] [INFO] [1621445679.071037031] [rclcpp]: signal_handler(signal_value=2)
[INFO] [fibonacci_action_server-1]: process has finished cleanly [pid 2053673]
-- run_test.py: return code 250
-- run_test.py: generate result file '/tmp/asd' with failed test
-- run_test.py: verify result file '/tmp/asd'
```
