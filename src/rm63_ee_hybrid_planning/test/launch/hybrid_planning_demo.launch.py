import launch
import os
import sys

from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import (
    generate_common_hybrid_launch_description,
    get_robot_description,
    get_robot_description_semantic,
    load_yaml,
)


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    common_launch = generate_common_hybrid_launch_description()
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    # Demo node
    common_hybrid_planning_param = load_yaml(
        "rm63_ee_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    # 启动了在 rm63_ee_hybrid_planning 包中的 hybrid_planning_demo_node.cpp定义的节点
    demo_node = Node(
        package="rm63_ee_hybrid_planning",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            get_robot_description(),
            get_robot_description_semantic(),
            common_hybrid_planning_param,
        ],
    )

    return launch.LaunchDescription(common_launch + [demo_node])
