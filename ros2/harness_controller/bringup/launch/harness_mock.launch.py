"""Bring the harness controller up against mock_components/GenericSystem.

Development plan Step 4: "Run against mock_components/GenericSystem first —
no simulator, real controller manager, real executor, real DDS." This launch
file is that step. Everything here is real except the physics.

Controller activation order is load-bearing and therefore explicit rather
than left to chance: harness_controller must be active and exporting its
reference interfaces BEFORE upstream_effort_controller can claim them.
Spawning them concurrently is a race that fails maybe one run in five, with
an error ("cannot find reference interface") that reads like a typo rather
than a scheduling problem.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _controller_params_with_golden_path(share_dir: str) -> str:
    """Write a copy of the controllers YAML with golden_joint_order_file filled in.

    The controller takes an absolute path to its golden joint-order file, and
    that path is only knowable after the package is installed. Injecting it
    here — rather than committing a machine-specific absolute path, or
    teaching the controller to resolve package:// URLs it would then also
    have to resolve on a robot with no ament index — keeps the checked-in
    config portable and the controller's file handling dumb.
    """
    config_path = os.path.join(share_dir, "bringup", "config", "harness_controllers.yaml")
    golden_path = os.path.join(share_dir, "bringup", "config", "golden_joint_order.yaml")

    with open(config_path, "r", encoding="utf-8") as handle:
        params = yaml.safe_load(handle)

    params["harness_controller"]["ros__parameters"]["golden_joint_order_file"] = golden_path

    merged = tempfile.NamedTemporaryFile(
        mode="w", suffix="_harness_controllers.yaml", delete=False, encoding="utf-8"
    )
    yaml.safe_dump(params, merged, default_flow_style=False)
    merged.close()
    return merged.name


def generate_launch_description():
    share_dir = get_package_share_directory("harness_controller")

    urdf_path = os.path.join(share_dir, "bringup", "urdf", "harness_mock.urdf")
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    controller_params = _controller_params_with_golden_path(share_dir)

    # controller_manager takes the robot description off the /robot_description
    # topic (latched by robot_state_publisher), not as a parameter — the
    # parameter path was removed after Humble.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controller_params],
    )

    # Downstream first: it must be exporting reference interfaces before
    # anything can claim them.
    harness_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["harness_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    upstream_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "upstream_effort_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Claims state interfaces only, so it races with nothing and can come up
    # whenever. Without it there is no /joint_states or /dynamic_joint_states,
    # and the gated effort is invisible to ordinary ROS tooling.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            harness_spawner,
            # Chain the upstream controller only once the harness is up.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=harness_spawner,
                    on_exit=[upstream_spawner],
                )
            ),
        ]
    )
