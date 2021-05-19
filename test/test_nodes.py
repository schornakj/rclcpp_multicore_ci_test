#!/usr/bin/env python3

# Command to run:
# taskset -c 0-1 python3 -u /opt/ros/rolling/share/ament_cmake_test/cmake/run_test.py /tmp/asd --package-name rclcpp_multicore_ci_test --env TEST_LAUNCH_DIR=/home/jschornak/workspaces/test_ws/src/rclcpp_multicore_ci_test/test TEST_EXECUTABLE=/home/jschornak/workspaces/test_ws/build/rclcpp_multicore_ci_test/test_fixture_node --command /home/jschornak/workspaces/test_ws/src/rclcpp_multicore_ci_test/test/test_nodes.py

import os
import sys
from launch import LaunchService, LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    test_executable = os.getenv("TEST_EXECUTABLE")

    test_action = ExecuteProcess(
        cmd=[test_executable], name="test_node", output="screen"
    )

    fib_server_node = Node(
        package="action_tutorials_cpp",
        executable="fibonacci_action_server",
        name="fibonacci_action_server",
        output="screen",
    )

    ld = LaunchDescription([])
    ld.add_action(fib_server_node)

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)

    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == "__main__":
    sys.exit(main())
